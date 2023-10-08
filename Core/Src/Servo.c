/*
 * Servo.c
 *
 *  Created on: Oct 7, 2023
 *      Author: gusta
 */

#include "Servo.h"

void setPWMAngle(ServoConfig_t servoConfig, float angle) {
	Scale_t angleScale;
	angleScale.min = -90;
	angleScale.max = 90;

	Scale_t pwmScale = __getPWMScale(servoConfig.timerConfig);

	float convertedToPWM = __convertScales(angleScale, pwmScale,
			__getCalibratedAngle(servoConfig.calibration, angle));
	setPWM(servoConfig.timerConfig.handle, servoConfig.timerConfig.channel,
			servoConfig.timerConfig.period, convertedToPWM);
}

void setPWMPercentage(ServoConfig_t servoConfig, float angle) {
	Scale_t angleScale;
	angleScale.min = 0;
	angleScale.max = 100;

	Scale_t pwmScale = __getPWMScale(servoConfig.timerConfig);

	float convertedToPWM = __convertScales(angleScale, pwmScale,
			__getCalibratedAngle(servoConfig.calibration, angle));
	setPWM(servoConfig.timerConfig.handle, servoConfig.timerConfig.channel,
			servoConfig.timerConfig.period, convertedToPWM);
}

Scale_t __getPWMScale(TimerConfig_t timerConfig) {
	Scale_t pwmScale;
	pwmScale.min = timerConfig.minDutyCyclePercentage
			* timerConfig.period;
	pwmScale.max = timerConfig.maxDutyCyclePercentage
				* timerConfig.period;

	return pwmScale;
}

float __convertScales(Scale_t from, Scale_t to, float point) {
	float deltaScaleFrom = from.max - from.min;
	float deltaScaleTo = to.max - to.min;

	float result = ((point - from.min) / deltaScaleFrom) * deltaScaleTo
			+ to.min;

	//if(result > to.max) return to.max;
	//if(result < to.min) return to.min;

	return result;
}

/*
 * Foi observado que há imprecisões no servo motor, de forma a ser necessário ajustar o valor enviado conforme a reta de calibração obtida
 */
float __getCalibratedAngle(ServoCalibration_t calibration, float desiredAngle) {
	return calibration.gain * desiredAngle + calibration.offset; // y = a*x + b
}

void setPWM(TIM_HandleTypeDef timer, uint32_t channel, uint32_t period,
		uint16_t pulseLength) {
	HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
	TIM_OC_InitTypeDef sConfigOC;
	timer.Init.Period = period; // set the period duration
	HAL_TIM_PWM_Init(&timer); // reinitialise with new period value
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulseLength; // set the pulse duration
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
	HAL_TIM_PWM_Start(&timer, channel); // start PWM generation
}
