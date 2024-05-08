/*
 * uart2.c
 *
 *  Created on: Oct 30, 2023
 *      Author: nguye
 */
#include "uart2.h"

uint8_t  SizeBuffer;


void UART2_Init(void){
	/*--------------------------------Configuration uart gpio pin------------------------------------------*/
	/*Enble clock access to port A*/
	RCC->AHB1ENR |= (1U<<0);
	/*Set PA2  & PA 3 mode to alternate function mode */
	GPIOA->MODER |= (0x2<<4);
	GPIOA->MODER |= (0x2<<6);
	/*Set PA2 & PA3 alternate function type to UART_TX (AF07)*/
	GPIOA->AFR[0] |= (0x7<<8);
	GPIOA->AFR[0] |= (0x7<<12);
	/*-------------------------------Configuration uart module -------------------------------------------*/
	/*Enable clock access to uart 2*/
	RCC->APB1ENR = (1U<<17);
	/*Configure baudrate*/
	/*9600- 16Mhz*/
	USART2->BRR |= (0x68<<4);			/*Clock input 16Mhz*/
	USART2->BRR |= (0x03<<0);			/*Baudrate = 9600bps*/
	/*Configure word length*/
	USART2->CR1 &=~(1U<<12);			/*8 bit Data */

	/*Configure number of stop bits*/
	USART2->CR2 &=~(0x3<<12);			/* 1 stop bit*/

	/*Configure transfer direction*/
	USART2->CR1 |= (1U<<3); 			/*Transmit*/
	USART2->CR1 |= (1U<<2);				/*Receiver*/

	/*Enable Interrupt when Data valid*/
	USART2->CR1 |=(1U<<5);

	/*Enable uart module*/
	USART2->CR1 |= (1U<<13);
}



void UART_Transmit( uint8_t* TX_Data, uint8_t TX_Size){
	uint8_t i = 0;
	while( i < TX_Size){
		/* Check Transmit Data Register Empty*/
		while(!(USART2->SR& (1<<7))){}
		/* Send data <=> Clear TXE Flag (Transmit Data Register empty)*/
		USART2->DR=(uint8_t)(*TX_Data & 0xFF);
		TX_Data++;
		i++;
	}
	/*Waiting transmit complete*/
	while(!(USART2->SR &(1<<6))){}
}

void UART_Receive(uint8_t *RX_Data) {
    uint8_t i = 0;
    SizeBuffer = 0;
    while (1) {
        /* Waitting Data Receive not empty <=> RXNE */
        while (!(USART2->SR & (1 << 5)));

        /* Clear RXNE */
        *RX_Data = (uint8_t)(USART2->DR & 0xFF);

        if (*RX_Data == '.') {
            SizeBuffer = i; // Đặt giá trị cho SizeBuffer trước khi kết thúc vòng lặp
            break;
        }

        RX_Data++;
        i++;
    }
}

