/*
 * File:   USART.c
 * Author: Jack
 *
 * Created on November 15, 2020, 2:05 PM
 */


#include <xc.h>
#include "LCDroutinesEasyPic.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pic18f87k22.h>

#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF

#define _XTAL_FREQ 16000000   //16 Mhz oscillator clock
#define STR_MAX 9

/******************************************************************************
 * Global variables
 ******************************************************************************/
char test = 0;
char tx_string[STR_MAX + 25];
//char error_str[STR_MAX + 25];
char tx_pos = 0;
char end_tx = 0;

char rx_string[STR_MAX];
char rx_buffer[STR_MAX];
char rx_pos = 0;
char new_rx = 0;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void init(void);
void init_USART(void);
void TxUsartHandler(void);
void RxUsartHandler(void);
void read_usart_str(void);
void write_usart_str(void);

/******************************************************************************
 * main()
 ******************************************************************************/
void main(void) {
    init();
    
    while(1){
        if (new_rx == 1){
            read_usart_str();
            write_usart_str();
            new_rx = 0;  
        }
    }
    
    return;
}


void init(){   
    RCONbits.IPEN = 1;              // Enable priority levels
    INTCONbits.GIEL = 1;            // Enable low-priority interrupts to CPU
    INTCONbits.GIEH = 1;            // Enable all interrupts
    INTCONbits.PEIE = 1;
    
    init_USART(); 
}


void init_USART(){
//    memset(rx_string, '\0', sizeof(rx_string));
    
    TRISCbits.TRISC7 = 1;           // RX
    TRISCbits.TRISC6 = 0;           // TX
    // TXSTA1 = 0b00100000;         // Tx on
    TXSTA1 = 0b00000000;            // Tx off
    BAUDCON1 = 0b00000000;
    SPBRG1 = 12;
    RCSTA1 = 0b10010000;
   
    PIE1bits.TX1IE = 1;
    PIE1bits.RC1IE = 1;
}


void read_usart_str(){
    if (strncmp(rx_string, "TEMP", 4) == 0){
        strncpy(tx_string, rx_string, sizeof(tx_string));
    }
    
    else if (strncmp(rx_string, "POT", 3) == 0){
        strncpy(tx_string, rx_string, sizeof(tx_string));
    }
    
    else if (strncmp(rx_string, "CONT_ON", 7) == 0){
        strncpy(tx_string, rx_string, sizeof(tx_string));
    }
    
    else if (strncmp(rx_string, "CONT_OFF", 8) == 0){
        strncpy(tx_string, rx_string, sizeof(tx_string));
    }
    
    else {   
        if (rx_string[STR_MAX - 1] != '\0' && rx_string[STR_MAX - 1] != '\n'){
            strcpy(tx_string, "Error: command too long\n");
            strncpy(tx_string, strcat(tx_string, rx_string), sizeof(tx_string));
        }
        
        else{
            strcpy(tx_string, "Error: unknown command, ");
            strncpy(tx_string, strcat(tx_string, rx_string), sizeof(tx_string));
        }
    }
    memset(rx_string, '\0', sizeof(rx_string));
}


void write_usart_str(){
    TXREG1 = tx_string[0];      // load first char
    end_tx = 0;                 // no longer at the end of a tx
    tx_pos = 1;                 // next index to look at
    TXSTA1bits.TXEN1 = 1;       // enable tx
}


void TxUsartHandler(){
    
    if (end_tx == 1) {
        TXSTA1bits.TXEN1 = 0;
    }
    
    else{
        TXREG1 = tx_string[tx_pos];     // should clear the flag by itself?
        
        if (tx_string[tx_pos] == '\n'){
            end_tx = 1;
        }
        
        ++tx_pos;
    }
}


void RxUsartHandler(){
    if (RCSTA1bits.FERR1 == 1){     // framing error
        char toss = RCREG1;
        RCSTA1bits.FERR1 = 0;
        return;
    }
    
    else if (RCSTA1bits.OERR1 == 1){
        RCSTA1bits.CREN1 = 0;
        RCSTA1bits.CREN1 = 1;
        RCSTA1bits.OERR1 = 0;
        return;
    }
    
    rx_buffer[rx_pos] = RCREG1;     // should clear the flag by itself?
    
    if (rx_buffer[rx_pos] == '\n'){
        strncpy(rx_string, rx_buffer, rx_pos+1);
        rx_pos = 0;
        new_rx = 1;               
    }
    
    else{
        ++rx_pos;
    }
}


void __interrupt() HiPriISR(void) {
    
    while(1) {
        if(PIR1bits.RC1IF) {
            RxUsartHandler();
            continue;
        }
        
        else if (PIR1bits.TX1IF)
            TxUsartHandler();

        break;      
    }
}

void __interrupt(low_priority) LoPriISR(void) 
{
    // Save temp copies of WREG, STATUS and BSR if needed.
    while(1) {
     
        // restore temp copies of WREG, STATUS and BSR if needed.
        break;     
    }
}