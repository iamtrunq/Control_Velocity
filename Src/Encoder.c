/*
 * PH0 : Channel 1
 * PH1 : Channel 2
 *
 * */
#ifndef ENCODER_C_
#define ENCODER_C_

#include<stm32f407xx.h>

void Read_Encoder_Init(void){
	/*Use PC1 and PC2 to read encoder signal from channel A and channel B*/
	/*Enable clock for port C*/
	RCC->AHB1ENR |= (1<<2);

	/*Configurate input mode*/
	GPIOC->MODER &=~(0x3<<2);	//PC1
	GPIOC->MODER &=~(0x3<<4);	//PC2

	RCC->APB2ENR |= (1<<14);
	/*Configurate Port to EXTI line*/
	SYSCFG->EXTICR[0] |= (0x2<<4);
	SYSCFG->EXTICR[0] |= (0x2<<8);

	/*EXTI with rising edge and falling edge*/
	EXTI->RTSR |= (0x3<<1);
	EXTI->FTSR |= (0x3<<1);

	/*Enable mask interrupt*/
	EXTI->IMR |= (0x3<<1);

	/*Configurate NVIC*/
	/*Clear pending*/
	//NVIC->IP[0] |= (1<< (EXTI0_IRQn%32));
	//NVIC->IP[0] |= (1<< (EXTI1_IRQn%32));
	NVIC->ICPR[0] |= (1U << (EXTI1_IRQn % 32));
	NVIC->ICPR[0] |= (1U << (EXTI2_IRQn % 32));
	/*Set priority*/
	NVIC_SetPriority(EXTI1_IRQn,0);
	NVIC_SetPriority(EXTI2_IRQn,0);

	/*Enable interrupt*/
	NVIC_EnableIRQ(EXTI1_IRQn);
	NVIC_EnableIRQ(EXTI2_IRQn);

}


#endif /* ENCODER_C_ */
