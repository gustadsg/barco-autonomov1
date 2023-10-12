/*
 * Servo.h
 *
 *  Created on: Oct 7, 2023
 *      Author: gusta
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "Pwm.h"

#define SERVO_MIN_ANGLE -90;
#define SERVO_MAX_ANGLE 90;
#define SERVO_MIN_PERCENT 0;
#define SERVO_MAX_PERCENT 100;

typedef struct {
	float min;
	float max;
} Scale_t;

typedef struct {
	float offset;
	float gain;
} ServoCalibration_t;

typedef struct {
	TIM_HandleTypeDef handle;
	uint32_t channel;
	uint32_t period;

	//	porcentagens de duty cycle para ângulos mínimo e máximo
	float minDutyCyclePercentage;
	float maxDutyCyclePercentage;
} ServoTimerConfig_t;

typedef struct {
	ServoTimerConfig_t timerConfig;
	ServoCalibration_t calibration;
} ServoConfig_t;

void setServoPWMAngle(ServoConfig_t servoConfig, float angle);
void setServoPWMPercentage(ServoConfig_t servoConfig, float angle);

float __convertScales(Scale_t from, Scale_t to, float point);
float __getCalibratedAngle(ServoCalibration_t calibration, float desiredAngle);
Scale_t __getPWMScale(ServoTimerConfig_t timerConfig);

#endif /* INC_SERVO_H_ */
