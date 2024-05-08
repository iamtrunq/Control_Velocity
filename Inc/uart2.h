/*
 * uart2.h
 *
 *  Created on: Oct 30, 2023
 *      Author: nguye
 */
#include <stdio.h>
#include <stdint.h>
#include <stm32f407xx.h>

#ifndef UART2_H_
#define UART2_H_
/* PA2 : TX
 * PA3 : RX
 * Baudrate: 19200
 * Stop bit : 1 bit
 * Size of Data : 8 bit
 */

void UART2_Init(void);
void UART2_Interrupt_Init(void);
void UART_Transmit( uint8_t* TX_Data, uint8_t TX_Size);
void UART_Receive(uint8_t *RX_Data);


#endif /* UART2_H_ */
