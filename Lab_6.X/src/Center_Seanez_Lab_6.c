// TODO: Acquisition time is off, the POT is changing the TEMP val on the
//       LCD
// TODO: Convert temp_val to actual temperature values in the LCD interrupt

/****** ASEN 4/5067 Lab 6 ******************************************************
 * Author: Jack Center and Marcela Seanez
 * Date  : November 10, 2020
 *
 * Updated for XC8
 * 
 * Description
 * On power up execute the following sequence:
 *      RD5 ON for 0.5s +/- 10ms then off
 *      RD6 ON for 0.5s +/- 10ms then off
 *      RD7 ON for 0.5s +/- 10ms then off
 * The following then occurs forever:
 *      RD4 blinks: 100ms +/- 10ms ON, then 900ms +/- 10ms OFF
 *      LCD Displays the following lines:
 *          'T=xx.x C'
 *          'PT=x.xxV'
 *      Where the 'x' is replaced by a digit in the measurement.
 *          Temperature data must be calculated / displayed with one digit to
 *          the right of the decimal as shown.  The sensor itself can have
 *          errors up to +/- 5 degrees Celsius.
 *          Potentiometer data must be calculated / displayed with two digits
 *          to the right of the decimal as shown.
 *          These measurements must be refreshed at LEAST at a frequency of 5Hz.
 *      USART Commands are read / executed properly. '\n' is a Line Feed char (0x0A)
 *          ASEN 4067:
 *              'TEMP\n'     - Transmits temperature data in format: 'XX.XC'
 *              'POT\n'      - Transmits potentiometer data in format: X.XXV'
 *          ASEN 5067: Same as ASEN 4067, plus two additional commands
 *              'CONT_ON\n'  - Begins continuous transmission of data over USART
 *              'CONT_OFF\n' - Ends continuous transmission of data over USART
 *
 *              Continuous transmission should output in the following format:
 *                  'T=XX.XC; PT = X.XXV\n'
 *      DAC is used to output analog signal onto RA5 with jumper cables. 
 *          ASEN 4067:
 *              Potentiometer voltage is converted from a digital value to analog 
 *              and output on the DAC. 
 *          ASEN 5067: 
 *              A 0.5 Hz 0-3.3V triangle wave is output on the DAC. 
 *******************************************************************************
 *
 * Program hierarchy 
 *
 * Mainline
 *   Initial
 *
 * HiPriISR (included just to show structure)
 *
 * LoPriISR
 *   TMR0handler
 ******************************************************************************/

#include <xc.h>
#include "LCDroutinesEasyPic.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pic18f87k22.h>


#define _XTAL_FREQ 16000000   //Required in XC8 for delays. 16 Mhz oscillator clock
#define _TEMP 0b00001101
#define _POT 0b00000001
#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

/******************************************************************************
 * Global variables
 ******************************************************************************/
short temp_val = 0;
short pot_val = 0;
char bufferH = 0;
char bufferL = 0;
short adc_val = 0;
char current_sensor = _POT;
char previous_sensor = _POT;
char flag_adc_reading = 0;
char flag_USART_rx = 0;
const unsigned short ccp4_time = 65000;           // 16ms worth of instructions

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void init(void);         
void init_lights(void);
void init_LCD(void);
void init_TMR0(void);
void init_TMR1(void);
void init_CCP4(void);
void init_ADC(void);
void init_USART(void);
void init_SPI(void);
void get_adc_reading(char*, const char*, const char *);
void read_USART(void);
void TMR0handler(void);     
void CCP4handler(void);
void read_ADC(void);


/******************************************************************************
 * main()
 ******************************************************************************/
void main() {
    init(); 
    char reading_count = 0;
    char throw_away = 2;            // measurements to throw away
    char measurements_max = 2;      // measurements to keep
    
    while(1) {
        // loop here and check our flags for actions

//        while (new_reading == 0) {}
        if (flag_adc_reading != 0){
            get_adc_reading(&reading_count, &throw_away, &measurements_max);
            flag_adc_reading = 0;
        }
        
        if (flag_USART_rx != 0){
            read_USART();
            flag_USART_rx = 0;
        }
    }
}

/******************************************************************************
 * Init()
 *
 * This subroutine performs all initializations of variables and registers.
 * It enables TMR0 and sets CCP1 for compare, and enables LoPri interrupts for
 * both.
 ******************************************************************************/
void init() {
    init_lights();      // Port D and start up routine
    init_LCD();         // Port B and start up routine
    
    RCONbits.IPEN = 1;  // Enable priority levels
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts
    
    init_TMR0();        // blink alive
    init_TMR1();        // update LCD
    init_CCP4();        // update LCD
    init_ADC();         // initialize ADC
}


void init_lights(){
    // Configure the IO ports
    TRISD  = 0b00001111;
    LATD = 0;
    
    LATDbits.LATD5 = 1;
    __delay_ms(500); 
    LATDbits.LATD5 = 0;
         
    LATDbits.LATD6 = 1;
    __delay_ms(500); 
    LATDbits.LATD6 = 0;
    
    LATDbits.LATD7 = 1;              
    __delay_ms(500);  
    LATDbits.LATD7 = 0;
}

void init_LCD(){
    // Configure the LCD pins for output. Defined in LCDRoutines.h
    LCD_RS_TRIS   = 0;              // 
    LCD_E_TRIS    = 0;
    LCD_DATA_TRIS = 0b11000000;     // Note the LCD is only on the upper nibble
                                    // The lower nibble is all inputs
    LCD_DATA_LAT = 0;           // Initialize LCD data LAT to zero
    // Initialize the LCD and print to it
    InitLCD();
    
    char LCDRow1[] = {0x80,'T', ' ', ' ', '=', ' ', 'x','x','.','x',0x00};
    char LCDRow2[] = {0xC0,'P','T', ' ', '=', ' ', 'x','.','x','x',0x00};
    DisplayC(LCDRow1);
    DisplayC(LCDRow2);
}

void init_TMR0(){
    //Initializing TMR0
    T0CON = 0b00000101;
    TMR0H = 0x24; 
    TMR0L = 0x46; 

    // Configuring Interrupts

    INTCON2bits.TMR0IP = 1;         // Assign low priority to TMR0 interrupt
    INTCONbits.TMR0IE = 1;          // Enable TMR0 interrupts
    T0CONbits.TMR0ON = 1;           // Turning on TMR0
}

void init_TMR1(){
    T1CON = 0b00110110;             // 16-bit, Fosc / 4, 8 pre scalar, off 
    TMR1L = 0;                      // Clearing TMR0 registers
    TMR1H = 0;

    CCPTMRS1 = 0b00000000;          // CCP4 -> TMR1
    
    PIE1bits.TMR1IE = 0;            // Disable interrupts
    T1CONbits.TMR1ON = 1;           // Turn on TMR1
}

void init_CCP4(){
  
    CCP4CON = 0b00001010;
    CCPR4L = (char)(ccp4_time & 0x00FF);
    CCPR4H = (char)((ccp4_time >> 8) & 0x00FF);
    
    IPR4bits.CCP4IP = 0;    // Low pri
    PIE4bits.CCP4IE = 1;    // enable
}

void init_ADC(){
    ADCON1 = 0b00000000;    //Configure ADCON1 for AVdd(GND) and AVss(3.3V)
    ADCON2 = 0b10010101;    //Configure ADCON2 for right justified; Tacq = 4Tad 
                            //and Tad = 16Tosc
    ANCON0bits.ANSEL3 = 1;  //Configure AN3 as analog input -- TEMP SENSOR
    TRISAbits.TRISA3 = 1;   //Configure TRIS register as INPUT RA3 -- TEMP SENSOR

    ANCON0bits.ANSEL0 = 1;  //Configure AN0 as analog input
    TRISAbits.TRISA0 = 1;
    
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    IPR1bits.ADIP = 0;
    
    ADCON0 = _POT;
    ADCON0bits.GO = 1;
}

void init_USART(){
    TRISC = 0x00;     //RC6 tx, RC7 rx
    TXSTA1 = 0b10100000;
    //RCSTA1;
    RCSTA1bits.SPEN = 1;
    BAUDCON1 = 0b00000000;
    SPBRGH1 = 0;
    SPBRG1 = 12;
}

void init_SPI(){
    TRISCbits.TRISC5 = 0;       //SD0 --> output (RC5)
    TRISCbits.TRISC4 = 1;       //SD1 --> input (RC4)
    TRISCbits.TRISC3 = 0;       //SCK --> output (RC3)
    
    SSP1STAT = 0b00000000;      //Initialize SSP1STAT
    SSP1STATbits.SMP = 1;
    SSP1STATbits.CKE = 0;       //CKE 0 for Mode(0,0) or 1 for Mode(1,1)
    SSP1STATbits.BF = 0;        //Clears automatically after BF=1 
    
    SSP1CON1bits.WCOL = 0;      //if BF=1 --> WCOL=1 must be CLEARED
    SSP1CON1bits.SSPOV = 0;
    SSP1CON1bits.SSPEN = 1;
    SSP1CON1bits.CKP = 1;       //CKP 1 for Mode(0,0) or 0 for Mode(1,1)
    // MSSP -- Need to figure out clock frequency
    

}

void get_adc_reading(char* count, const char* throw, const char* meas_max){
    if (*count < *throw){   
            // discard first reading
            *count += 1;           
        }
        
        else if (current_sensor == _POT){
            // update the pot value with adc value
            pot_val = adc_val;
            *count += 1;
        }
        
        else if (current_sensor == _TEMP){
            // update the temp value with adc value
            temp_val = adc_val;
            *count += 1;
        }
        
        if (*count > *meas_max){
            // swap sensors
            if (current_sensor == _TEMP) {
                current_sensor = _POT;
            }
            
            else if (current_sensor == _POT){
                current_sensor = _TEMP;
            }
//            __delay_us(6);
            *count = 0;
        }
}

void read_USART(){
    
}
/******************************************************************************
 * HiPriISR interrupt service routine
 *
 * Included to show form, does nothing
 ******************************************************************************/

void __interrupt() HiPriISR(void) {
    
    while(1) {
        if(INTCONbits.TMR0IF ) {
            TMR0handler();
            continue;
        }

        break;      
    }
}

/******************************************************************************
 * LoPriISR interrupt service routine
 *
 * Calls the individual interrupt routines. It sits in a loop calling the required
 * handler functions until until TMR0IF and CCP1IF are clear.
 ******************************************************************************/

void __interrupt(low_priority) LoPriISR(void) 
{
    // Save temp copies of WREG, STATUS and BSR if needed.
    while(1) {
        if( PIR4bits.CCP4IF ) {   // change to CCP4
            CCP4handler();
            continue;
        }
        
        else if( PIR1bits.ADIF ){    //ADC acquisition finished
            read_ADC();
            continue;
        }
        
        // restore temp copies of WREG, STATUS and BSR if needed.
        break;     
    }
}


/******************************************************************************
 * TMR0handler interrupt service routine.
 *
 * Handles Alive LED Blinking via counter
 ******************************************************************************/
void TMR0handler() {
    LATDbits.LATD4 = ~LATDbits.LATD4;
    
    if( LATDbits.LATD4 ){  
        TMR0H = 0x3C; 
        TMR0L = 0xB0;               // Loading times for 100ms ON
        T0CONbits.T0PS = 0b010;     // Loading pre-scaler of 8
    }
    else {
        TMR0H = 0x24; 
        TMR0L = 0x46;               // Loading times for 900ms
        T0CONbits.T0PS = 0b101;     // Loading pre-scaler of 64
    }
    
    INTCONbits.TMR0IF = 0;      //Clear flag and return to polling routine
}


/******************************************************************************
 * CCP4handler interrupt service routine.
 *
 * Handles updating the LCD approximately every 130ms
 ******************************************************************************/
void CCP4handler(){
    
    // 12^2 bins from 0 to 3.3V -> current bin * 3.3/4096 = volts
    short pot_val_temp = 0.0806 * pot_val;  
    short temp_val_temp = 0.806 * temp_val;   
    
    char pot_str[4]; 
    char temp_str[4];
    
    sprintf(pot_str, "%d", pot_val_temp);
    sprintf(temp_str, "%d", temp_val_temp);

    if (pot_val_temp < 10){
        // value does not take up 3 digits, need to add leading 0
        pot_str[2] = pot_str[0];
        pot_str[1] = '0';
        pot_str[0] = '0';
    }
    
    else if (pot_val_temp < 100){
        // value does not take up 3 digits, need to add leading 0
        pot_str[2] = pot_str[1];
        pot_str[1] = pot_str[0];
        pot_str[0] = '0';
    }
    
    char temp_LCD[6] = {0x85, temp_str[0], temp_str[1],'.',temp_str[2],0x00};
    char pot_LCD[6] = {0xC5, pot_str[0], '.', pot_str[1] ,pot_str[2],0x00};
    
    DisplayC(temp_LCD);
    DisplayC(pot_LCD); 
    
    CCPR4L += (char)(ccp4_time & 0x00FF);
    CCPR4H += (char)((ccp4_time >> 8) & 0x00FF);
    PIR4bits.CCP4IF = 0;
    
    // TODO: Testing, delete when update functions are added
//    temp_val += 1;
//    pot_val += 1;
}

void read_ADC(){
    bufferH = ADRESH;
    bufferL = ADRESL;                   //Save low/high values of ADC
    adc_val = (bufferH << 8) | bufferL; //Concatenate high and low bytes     
    ADCON0 = current_sensor;            //Configure ADCON0 to read current sensor;
//     if current_sensor != previous_sensor => wait 2us (~= 2.45 - 2*.25)
//    if (current_sensor != previous_sensor){
//        ADCON0 = current_sensor;            //Configure ADCON0 to read current sensor;
//        previous_sensor = current_sensor;
//        __delay_us(50);
//    }

    ADCON0bits.GO = 1;                  //Start acquisition then conversion
    PIR1bits.ADIF = 0;                  //Clear ADC flag
    flag_adc_reading = 1;
}
