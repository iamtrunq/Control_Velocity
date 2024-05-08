#include <stm32f407xx.h>

#define SYSTICK_LOAD_VALUE		16000
#define CTRL_ENABLE				(1U<<0)
#define CTRL_CLKSRC				(1U<<2)
#define	CTRL_COUNTFLAG			(1U<<16)

void SYS_TICK_Delay(int ms){
	/*Reaload with number of clocks per millisecond*/
	SysTick->LOAD = SYSTICK_LOAD_VALUE;

	/*Clear systick current value register*/
	SysTick->VAL = 0;

	/*Enable systick and select internal clock source*/
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;

	for(int i = 0;i < ms; i++){
		/*Wait until the countflag is set*/

		while( (SysTick->CTRL & CTRL_COUNTFLAG) == 0 ){}
	}
	SysTick->CTRL = 0;
}
