/*
 * PWM.h
 *
 *  Created on: Nov 6, 2023
 *      Author: nguye
 */

#ifndef PWM_H_
#define PWM_H_

#include <stm32f407xx.h>

void PWM_Init();

void SetPWM(uint8_t PWM);

#endif /* PWM_H_ */
