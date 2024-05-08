#include <stm32f407xx.h>

void GPIO_Toggle_Init(){
	RCC->AHB1ENR |= (1<<3);   //ENABLE clock for port D
	GPIOD->MODER |= (0x55<<24);  // Output for D12 ,D13, D14, D15
	GPIOD->ODR &=~ (0xF<<12);
}

void Toggle(){
	GPIOD->ODR ^= (1<<14);
	GPIOD->ODR ^= (1<<15);
	GPIOD->ODR ^= (1<<12);
	GPIOD->ODR ^= (1<<13);
}
