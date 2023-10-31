/*
 * DCMotor.h
 *
 *  Created on: Oct 11, 2023
 *      Author: João
 */

#ifndef INC_DCMOTOR_H_
#define INC_DCMOTOR_H_

#include "Pwm.h"

#define DC_MOTOR_MIN_PERCENT 0;
#define DC_MOTOR_MAX_PERCENT 100;

typedef struct {
	TIM_HandleTypeDef handle;
	uint32_t channel;
	uint32_t period;
} DCMOTOR_TimerConfig_t;

void DCMOTOR_SetSpeedPercentage(DCMOTOR_TimerConfig_t timerConfig, float percentage);

#endif /* INC_DCMOTOR_H_ */
