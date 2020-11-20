/*
 * File:   USART.c
 * Author: Jack
 *
 * Created on November 15, 2020, 2:05 PM
 */


#include <xc.h>
//#include "LCDroutinesEasyPic.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <pic18f87k22.h>
#include <USART.h>

//#pragma config FOSC=HS1, PWRTEN=ON, BOREN=ON, BORV=2, PLLCFG=OFF
//#pragma config WDTEN=OFF, CCP2MX=PORTC, XINST=OFF
//
//#define _XTAL_FREQ 16000000   //16 Mhz oscillator clock
#define STR_MAX 9

/******************************************************************************
 * Global variables
 ******************************************************************************/
static char tx_string[STR_MAX + 25];
static char tx_pos = 0;
static char end_tx = 0;

static char rx_string[STR_MAX];
static char rx_buffer[STR_MAX];
static char rx_pos = 0;
extern char new_rx; 
extern char ccp5_x;
extern short pot_val;
extern short temp_val;
extern char cont_on;



void init_USART(){
    TRISCbits.TRISC7 = 1;           // RX
    TRISCbits.TRISC6 = 0;           // TX
    // TXSTA1 = 0b00100000;         // Tx on
    TXSTA1 = 0b00000000;            // Tx off
    BAUDCON1 = 0b00000000;
    SPBRG1 = 12;
    RCSTA1 = 0b10010000;
   
    PIE1bits.TX1IE = 1;
    IPR1bits.TX1IP = 0;
    
    PIE1bits.RC1IE = 1;
    IPR1bits.RC1IP = 1;
}


void read_usart_str(){
    
    if (strncmp(rx_string, "CONT_OFF", 8) == 0){
        ccp5_x = 0;
        PIE4bits.CCP5IE = 0;    // disable
        PIR4bits.CCP5IF = 0;    // clear flag
//        strncpy(tx_string, "\n",1);
        cont_on = 0;
        memset(rx_string, '\0', sizeof(rx_string));
    }
    else if (cont_on == 1){
        char temp_tx[8];
        convert_temp_to_tx(temp_tx, 8);
        strncpy(tx_string, temp_tx, 6);
        
        char space[] = " ";
        strcat(tx_string, space);
        
        convert_pot_to_tx(temp_tx, 8);
        strncpy(tx_string, strcat(tx_string, temp_tx), 8);
    }
    
    else if (strncmp(rx_string, "TEMP", 4) == 0){
        char temp_tx[8];
        convert_temp_to_tx(temp_tx, 8);
        strncpy(tx_string, temp_tx, sizeof(tx_string));
        memset(rx_string, '\0', sizeof(rx_string));
    }
    
    else if (strncmp(rx_string, "POT", 3) == 0){
        char temp_tx[8];
        convert_pot_to_tx(temp_tx, 8);
        strncpy(tx_string, temp_tx, sizeof(tx_string));
        memset(rx_string, '\0', sizeof(rx_string));
    }
    
    else if (strncmp(rx_string, "CONT_ON", 7) == 0){
        char temp_tx[8];
        convert_temp_to_tx(temp_tx, 8);
        strncpy(tx_string, temp_tx, 6);
        
        char space[] = " ";
        strcat(tx_string, space);
        
        convert_pot_to_tx(temp_tx, 8);
        strncpy(tx_string, strcat(tx_string, temp_tx), 8);

        cont_on = 1;
        PIE4bits.CCP5IE = 1;    // enable
        memset(rx_string, '\0', sizeof(rx_string));
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
        
        memset(rx_string, '\0', sizeof(rx_string));
    }
}


void write_usart_str(){
    if (tx_string[0] != '\0'){
        TXREG1 = tx_string[0];      // load first char
        end_tx = 0;                 // no longer at the end of a tx
        tx_pos = 1;                 // next index to look at
        TXSTA1bits.TXEN1 = 1;       // enable tx
    }
}


void TxUsartHandler(){
    
    if (end_tx == 1) {      //|| tx_string[tx_pos] == '\0'
        TXSTA1bits.TXEN1 = 0;
        memset(tx_string, '\0', sizeof(tx_string));
        
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
    
    rx_buffer[rx_pos] = RCREG1;    
    
    if (rx_buffer[rx_pos] == '\n'){
        strncpy(rx_string, rx_buffer, rx_pos+1);
        rx_pos = 0;
        new_rx = 1;               
    }
    
    else{
        ++rx_pos;
    }
}


void convert_temp_to_tx(char* val_ptr, int len){
    short temp_val_temp = 0.806 * temp_val; 
    char temp_str[4];
    sprintf(temp_str, "%d", temp_val_temp);    
    char display[] = {'T','=',temp_str[0],temp_str[1],'.',temp_str[2],'\n',0x00};
    
    for (int i = 0; i < len; ++i){
        *(val_ptr + i) = display[i];
    }
}

void convert_pot_to_tx(char* val_ptr, int len){
    short pot_val_temp = 0.0806 * pot_val; 
    char pot_str[4]; 
    sprintf(pot_str, "%d", pot_val_temp);
    
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
    
    char display[] = {'P','=',pot_str[0],'.',pot_str[1],pot_str[2],'\n',0x00};
    
    for (int i = 0; i < len; ++i){
        *(val_ptr + i) = display[i];
    }
}