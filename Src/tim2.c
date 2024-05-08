#include <tim2.h>
#include <stm32f407xx.h>
#define TIM1_EN		(1U<<0)


void TIM1_Init(void){

	/*Enable clock access to tim2*/
	RCC->APB2ENR = TIM1_EN;
	/*Set prescaler value*/
	TIM1->PSC = 800-1;    //16.000.000/800=20.000
	/*Set auto - reload value*/
	//TIM2->ARR = 100 -1 ; //20.000/200 = 200 -> 5ms
	TIM1->ARR = 200-1;
	/*Clear counter*/
	TIM1->CNT = 0;
	/*Clear interrupt flag*/
	TIM1->SR &=~(1U<<0);
	/*Configurate mode for timer*/
	/*No effect on the output*/
	TIM1->EGR |= (1<<0);
	/*Enable timer*/
	TIM1->CR1 |=(1<<0);

	/*Enable interrupt*/
	TIM1->DIER |=(1<<0);
}
void TIM1_Interrupt_Init(void){

	/*Clear pending  Pos : 28*/
	NVIC->ICPR[0] |= 1 << (TIM1_UP_TIM10_IRQn % 32);
	/*Set priority*/
	NVIC->IP[TIM2_IRQn] = 0x01;

	//NVIC_SetPriority(TIM2_IRQn, 2);
	//NVIC_EnableIRQ( TIM1_UP_TIM10_IRQn );
	NVIC->ISER[0] |= 1 << (TIM1_UP_TIM10_IRQn % 32);

}
