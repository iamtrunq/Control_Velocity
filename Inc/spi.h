/*
 * spi.h
 *
 *  Created on: Nov 25, 2023
 *      Author: nguye
 */

#ifndef SPI_H_
#define SPI_H_
#include "stm32f407xx.h"
#define APB1ENR_SPI2EN		(1U<<14)
#define AHB1ENR_GPIOB_EN	(1U<<1)

#define CR1_BIDIMODE		(1U<<15)
#define CR1_BIDIOE			(1U<<14)
#define CR1_DFF				(1U<<11)
#define CR1_RXONLY			(1U<<10)
#define CR1_SSM				(1U<<9)
#define CR1_LSBFIRST		(1U<<7)
#define CR1_SPE				(1U<<6)
#define CR1_MSTR			(1U<<2)
#define CR1_CPOL			(1U<<1)
#define CR1_CPHA			(1U<<0)

#define CR2_TXEIE			(1U<<7)
#define CR2_RXNEIE			(1U<<6)
#define CR2_ERRIE			(1U<<5)
#define CR2_FRF				(1U<<4)
#define CR2_SSOE			(1U<<2)
#define CR2_TXDMAEN			(1U<<1)
#define CR2_RXDMAEN			(1U<<0)

#define SR_FRE				(1U<<8)
#define SR_BSY				(1U<<7)
#define SR_OVR				(1U<<6)
#define SR_MODF				(1U<<5)
#define SR_CRCERR			(1U<<4)
#define SR_UDR				(1U<<3)
#define SR_CHSIDE			(1U<<2)
#define SR_TXE				(1U<<1)
#define SR_RXNE				(1U<<0)
void SPI2_Init(void);
void SPI_Transmit( uint8_t* TX_Data, uint8_t TX_Size);
void SPI_Receive(uint8_t* RX_Data, uint8_t RX_Size);
void SPI_Transmit_16bits( uint8_t* TX_Data, uint16_t TX_Size);

#endif /* SPI_H_ */
