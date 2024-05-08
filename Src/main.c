/*
 * Control Velocity using STM32F407
 * 1 Using UART to send desired velocity down STM32
 * 2 STM32 will receive Desired velocity, after using PID  algorithm to control velocity
 * 3 Current velocity will send to LED module (MAX7219) via SPI protocol
 * 4 Parameters of PID are not optimize
 *
 * Wiring diagram
 * MAX7219:
 * VCC	:	5v
 * GND	:	GND
 * DIN	:	PB15
 * CS	:	PD9
 * CLK	:	PB11
 *
 * L298:
 * +12V	: 	12V DC
 * GND	: 	0V
 * IN3	:	PA4
 * IN4	:	PA5
 * ENB	:	PC6 (PWM)
 * OUT3	: 	VCC Motor
 * OUT4	:	GND Motor
 *
 * Motor: 	Encoder channel A : PC0
 * Motor: 	Encoder channel B : PC1
 *
 * UART:
 * TX	:	PA2
 * RX	:	PA3
 * */
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stm32f407xx.h>
#include <uart2.h>
#include <stdio.h>
#include "PWM.h"
#include "GPIO.h"
#include "systick.h"
#include "tim2.h"
#include "Encoder.h"
#include "spi.h"
#include "string.h"

uint8_t NoDecode[11]={0x7E,0x30,0x6D,0x79,0x33,0x5B,0x5F,0x70,0x7F,0x7B,0x00};
uint8_t PreviousState;
int16_t	CountValue=0;
uint16_t CntVel;
int32_t PosCnt;
uint32_t pointer;
float  CurVel;
int pwm;
float Ts,anpha,Kb;
int16_t RealVel,HILIM,LOLIM;
int32_t Cnttmp;
volatile uint8_t count;
extern uint8_t SizeBuffer;
uint8_t Data[4];
uint16_t SetPoint= 0;


void SPI_Transmit_16bits( uint8_t* TX_Data, uint16_t TX_Size);
void Max7219_init();
void HELLO();
void SendData(uint8_t Address, uint8_t Data);
void Put_Number_Left(uint16_t value);
void NVIC_UART2(void);


int main(void)
{
	NVIC_UART2();
	UART2_Init();
	SPI2_Init();
	Max7219_init();
	GPIO_Toggle_Init();
	HELLO();
	Read_Encoder_Init();
	PWM_Init();
	TIM1_Init();
	TIM1_Interrupt_Init();

	/*A4 & A5 is dir pin*/
	/*Out mode*/
	GPIOA->MODER |=(0x1<<8);
	GPIOA->MODER |=(0x1<<10);
	/*Reset State*/
	GPIOA->ODR &=~ (1<<4);
	GPIOA->ODR &=~ (1<<5);

	while(1)
	{


	}


}
/*PID  algorithm */
int PID(float Desired, float Current , float p_coef, float i_coef, float d_coef){
	static float err_p = 0;
	static float dterm_f_p=0;
	static float iterm_p=0;
	static float err_sat=0;


	float err, err_windup;
	float pterm, dterm, dterm_f, iterm;


	int16_t pidout;


	HILIM = 100; LOLIM = 0;
	err=Desired-Current;

	// Khau P
	pterm=p_coef*err;

	// Khau D + Loc
	dterm=d_coef*(err-err_p)/Ts;
	dterm_f=(1-anpha)*dterm_f_p + anpha*dterm;


	//Khau I
	err_windup=i_coef*err + Kb*err_sat;
	iterm=iterm_p + Ts*err_windup;

	iterm_p = iterm;
	dterm_f_p = dterm_f;
	err_p = err;
	//PID :
	pidout=pterm + dterm_f + iterm;

	// Bao hoa:
	if(pidout > HILIM){
		pidout = 100;
		err_sat = HILIM - pidout;
	}else if(pidout < LOLIM){
		pidout = 0;
		err_sat =LOLIM - pidout;
	}else{
		pidout=pidout;
		err_sat = 0;
	}
	return pidout;

}

/*Encoder channel A */
void EXTI1_IRQHandler(void){
	if(EXTI->PR & (1U<<1)){
		uint8_t State1;
		State1 = ((GPIOC->IDR & (1<<1)) ? 1 : 0) | ((GPIOC->IDR & (1<<2)) ? 2 : 0);
		switch (State1) {
			case 0:
				if(PreviousState==1) CountValue++;
				else CountValue--;
				break;
			case 1:
				if(PreviousState==3) CountValue++;
				else CountValue--;
				break;
			case 2:
				if(PreviousState==0) CountValue++;
				else CountValue--;
				break;
			case 3:
				if(PreviousState==2) CountValue++;
				else CountValue--;
				break;
		}
		PreviousState = State1;
		CntVel++;
		if (CountValue>=105*4) {
			CountValue = 0;
			PosCnt++;
		}
		else if	(CountValue<=-105*4) {
			CountValue = 0;
			PosCnt--;
		}
	}
	/*Clear Pending*/
	EXTI->PR |= (1<<1);

}

/*Encoder channel B*/
void EXTI2_IRQHandler(void){
	if(EXTI->PR & (1U<<2)){
		uint8_t State0;
		State0 = ((GPIOC->IDR & (1<<1)) ? 1 : 0) | ((GPIOC->IDR & (1<<2)) ? 2 : 0);
		switch (State0) {
			case 0:
				if(PreviousState==1) CountValue++;
				else CountValue--;
				break;
			case 1:
				if(PreviousState==3) CountValue++;
				else CountValue--;
				break;
			case 2:
				if(PreviousState==0) CountValue++;
				else CountValue--;
				break;
			case 3:
				if(PreviousState==2) CountValue++;
				else CountValue--;
				break;
			}
		PreviousState = State0;
		CntVel++;
		if (CountValue>=105*4) {
			CountValue = 0;
			PosCnt++;
		}
		else if	(CountValue<=-105*4) {
			CountValue = 0;
			PosCnt--;
		}
	}
	/*Clear Pending*/
	EXTI->PR |= (1<<2);

}
/*Timer 5ms to read velocity and this is sample time*/
void TIM1_UP_TIM10_IRQHandler(){
	/*Sample time = 5ms*/
	Cnttmp = CntVel;
	CntVel = 0;
	RealVel = (Cnttmp*200)/7;										//RPM
	CurVel = (3.14*Cnttmp*20)/21;								//rad/s  (M*2PI)/(N*T)
	count++;

	/*After 4x25 = 100 ms then send value to led because sample time is short*/
	if (count == 25){
		count = 0;
		Put_Number_Left(RealVel);
	}
	Ts=0.005;
	anpha = 0.9;
	Kb=1;

	//Put_Number_Left(RealVel);
	pwm=PID(SetPoint,RealVel,0.5,0.9,0);
	SetPWM(pwm);
	GPIOA->ODR |= (1<<4);
	//GPIOA->ODR &=~(1<<5);
	/* Clear pending*/
	TIM1->SR &=~ (1<<0);
}

void NVIC_UART2(void){
	/*Clear Pending*/
	NVIC->ICPR[USART2_IRQn/32] |=(1U << (USART2_IRQn % 32));
	/*Enable interrupt */
	NVIC->ISER[USART2_IRQn/32] |=(1U << (USART2_IRQn % 32));
	/*Set priority 1*/
	NVIC->IP[USART2_IRQn/4] |= (0x01 << (8 * (USART2_IRQn%4) ) );
}


/*Interrupt UART2 handling*/
void USART2_IRQHandler(){
	/*Check flag status interrupt*/
	if(USART2->SR & (1<<5)){
		/*Clear Data*/
		for (int i = 0; i < 4; i++) {
		    Data[i] = 0;
		}
		/*Receive data and then transmit backward*/
		UART_Receive(Data);
		sscanf(Data,"%d.",&SetPoint);
		GPIOD->ODR ^=(1<<14);
		GPIOD->ODR ^=(1<<13);


		/*Transmit data */
		//UART_Transmit(Data,SizeBuffer);

	}
	USART2->SR &=~(1<<5);			//Clear Pending
}
void SendData(uint8_t Address, uint8_t Data){

	uint8_t Buffer[2];
    Buffer[0] = Address;
    Buffer[1] = Data;
    SPI_Transmit_16bits(Buffer,2);

}
void Max7219_init(){
    /* Set  no decode mode: 0x0000 */
	SendData(0x09,0x00);
    /* Set intensity: 0x0A09 */
	SendData(0x0A,0x09);
    /* Set scanlimit */
	SendData(0x0B,0x07);
    /* No shutdown, turn off display test */
	SendData(0x0C,0x01);
	SendData(0x0F,0x00);
}
void HELLO(){
	uint8_t mssv[8] = {0x08,0x37,0x4F,0x0E,0x0E,0x7E,0x08,0x00};
    for(int i=0;i<8;i++){
        SendData(i+1, mssv[7-i]);
    }
	SYS_TICK_Delay(500);

	/*Clear Display*/
    for(int i=0;i<8;i++){
        SendData(i+1, 0x00);
    }

}

void Put_Number_Left(uint16_t value){
	uint8_t donvi;
	uint8_t chuc;
	uint8_t tram;
	uint8_t nghin;
	if(value < 10){
		donvi = value;
		SendData(0x01,NoDecode[donvi]);
		SendData(0x02,NoDecode[11]);
		SendData(0x03,NoDecode[11]);
		SendData(0x04,NoDecode[11]);
	}else if(value < 100 ){
		chuc = value / 10;
		donvi = value % 10;
		SendData(0x01,NoDecode[donvi]);
		SendData(0x02,NoDecode[chuc]);
		SendData(0x03,NoDecode[11]);
		SendData(0x04,NoDecode[11]);
	}else if(value < 1000){
		tram = value / 100;
		chuc = (value - tram*100) /10;
		donvi = (value - tram*100) % 10;

		SendData(0x01,NoDecode[donvi]);
		SendData(0x02,NoDecode[chuc]);
		SendData(0x03,NoDecode[tram]);
		SendData(0x04,NoDecode[11]);
	}else{
		nghin = value / 1000;
		tram = (value - nghin*1000) / 100;
		chuc = (value - nghin*1000 - tram*100) / 10;
		donvi = (value - nghin*1000 - tram*100) % 10;

		SendData(0x01,NoDecode[donvi]);
		SendData(0x02,NoDecode[chuc]);
		SendData(0x03,NoDecode[tram]);
		SendData(0x04,NoDecode[nghin]);

	}

}

