#include "stm32f407xx.h"
#include "spi.h"


void SPI2_Init(void){
	/* PB12: NSS
	 * PB13: SCK
	 * PB14: MISO
	 * PB15: MOSI
	 * */
	/*Enable clock for SPI2*/
	RCC->APB1ENR |= APB1ENR_SPI2EN;

	/*Enable Clock for port B*/

	RCC->AHB1ENR |= AHB1ENR_GPIOB_EN;

	/*Configuates mode for port PB12 PB13 PB14*/
	/*General purpose output mode*/
	/*NSS: Output*/
	GPIOB->MODER |= (0x02<<24);
	/*SCK: Output*/
	GPIOB->MODER |= (0x02<<26);
	/*MISO : Input*/
	GPIOB->MODER |= (0x02<<28);
	/*MOSI: Output*/
	GPIOB->MODER |= (0x02<<30);


	/*Output push pull*/
	GPIOB->OTYPER  |= (0x0<<12);

	/*Speed for port B: Medium*/
	GPIOB->OSPEEDR |= (0x55<<24);

	/*Alternate function : ALT5*/
	GPIOB->AFR[1] |= (0x5555<<16);

	/*<---------SPI Configuration----------->*/
	/*Data Frame : 16bit*/
	//SPI2->SPI_CR1 &=~ (1<<11);
	SPI2->CR1 |= CR1_DFF;

	/*Software slave management: Control by hardware*/
	SPI2->CR1 &=~ CR1_SSM;

	/*Frame format: MBS first*/
	SPI2->CR1 &=~ CR1_LSBFIRST;

	/*Baurate speed: fAHB2/2 = 16/2=8Mhz */
	SPI2->CR1 &=~ (0x07<<3);

	/*Master selection mode*/
	SPI2->CR1 |= CR1_MSTR;

	/*Clock polarity CPOL:1*/
	SPI2->CR1 &=~ CR1_CPOL;

	/*Clock phase CPHA:1*/
	SPI2->CR1 &=~ CR1_CPHA;

	/*Control Register 2*/
	/*No configuration interrupt mode*/
	/*Frame format*/
	SPI2->CR2 &=~ CR2_FRF;

	/*SSOE : SS output enable. Enable in master mode */
	//SPI2->SPI_CR2 |= (1<<2);

	/*Enable SPI peripheral*/
	SPI2->CR1|= CR1_SPE;

	//RCC->RCC_APB1LPENR &=~(1<<14);
	RCC->AHB1ENR |= (1<<3);
	GPIOD->MODER|=(0x01<<18); //Output mode general purpose
	GPIOD->ODR |= (1<<9); // Pull up

}


void SPI_Transmit( uint8_t* TX_Data, uint8_t TX_Size){
	uint8_t i = 0;
	while( i < TX_Size){
		/* Check transmit buffer empty*/
		while( !(SPI2->SR & SR_TXE) );
		/* Send data */
		SPI2->DR =TX_Data[i];
		/*Wait SPI2 not busy: Transmit done*/
		while( !(SPI2->SR & SR_BSY) );
		i++;
	}
}
void SPI_Transmit_16bits( uint8_t* TX_Data, uint16_t TX_Size){
	/* Remember set bit Frame Data SPI2->CR1 |=(1<<11) */
	GPIOD->ODR &=~(1<<9);
	uint16_t i = 0;
	while(i< TX_Size){
		/* Check transmit buffer empty*/
		while( !(SPI2->SR & (1<<1)) );
		/* Send data */
		uint16_t temp = ((TX_Data[i]<<8) | (TX_Data[i+1]<<0)) ;
		SPI2->DR = temp;
		/*Wait SPI2 not busy: Transmit done*/
		while( !(SPI2->SR & SR_BSY) );
		i++;
		i++;
	}
	GPIOD->ODR |= (1<<9);
}
void SPI_Receive(uint8_t* RX_Data, uint8_t RX_Size){
	/* Remember reset bit Frame Data SPI2->CR1 &=~(1<<11) */
	while(RX_Size>0){
		/*Send dummy Data*/
		SPI2->DR = '0';
		/*Waiting SPI not busy*/
		while(SPI2->SR & SPI_SR_BSY);
		/*Waiting Receive buffer empty*/
		while(! (SPI2->SR & SPI_SR_RXNE) );
		*(RX_Data++)=SPI2->DR;
		RX_Size--;
	}
}
