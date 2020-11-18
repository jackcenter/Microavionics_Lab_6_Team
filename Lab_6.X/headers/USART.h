/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef USART_H
#define	USART_H

#include <xc.h> // include processor files - each processor file is guarded.  

void init_USART(void);
void TxUsartHandler(void);
void RxUsartHandler(void);
void read_usart_str(void);
void write_usart_str(void);

#endif	/* USART_H */

