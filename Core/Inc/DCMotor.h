/*
 * DCMotor.h
 *
 *  Created on: Oct 11, 2023
 *      Author: João
 */

#ifndef INC_DCMOTOR_H_
#define INC_DCMOTOR_H_

#include "Pwm.h"

#define MOTOR_MIN_PERCENT 0;
#define MOTOR_MAX_PERCENT 100;

void setDCMotorPWMPercentage(float percentage);

#endif /* INC_DCMOTOR_H_ */
