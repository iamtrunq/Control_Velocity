#include <stm32f407xx.h>


/*
 * PC 6

 *
 *
 * */
void PWM_Init(void){
	/*Enable clock for port C*/
	RCC->AHB1ENR |= (1U<<2);

	/*Alternate Mode*/
	GPIOC->MODER |=(0x2<<12);

	/*Alternate Function*/
	GPIOC->AFR[0] |= (0x2<<24);

}



void SetPWM(uint8_t PWM){
	/*
	 * Channel 1: PC6

	 * */

	/*Enable clock for Timer 3*/
	RCC->APB1ENR |=(1U<<1);
	/* Prescaler Value : = 2 = > 16.000.000/2 = 8.000.000*/
	TIM3->PSC = 8 - 1;  // 16.000.000 / 8 = 2.000.000
	/*Auto Reaload Register : = 100 => F = 2.000.000 / 100 = 10.000 Hz */
	TIM3->ARR = 100 - 1;

	/*Clear counter*/
	TIM3->CNT = 0;


	/*Enable PWM mode for TIM2 channel */
	TIM3->CCMR1 = 0x6060;		// Channel 3
	TIM3->CCMR2 = 0x6060;		// Channel 4

	/*OUT*/
	/*Channel 1: PC6*/
	TIM3->CCR1 = (PWM*100)/100 ;


	/*Enable output capture for channel*/
	TIM3->CCER |= (1U<<0); 			// Channel 1


	/*Enable Timer*/
	TIM3->CR1 |= (1U<<0);


}
