/*
 * Servo.c
 *
 *  Created on: Oct 7, 2023
 *      Author: gusta
 */

#include "Servo.h"

void setServoPWMAngle(ServoConfig_t servoConfig, float angle) {
	Scale_t angleScale;
	angleScale.min = SERVO_MIN_ANGLE
	;
	angleScale.max = SERVO_MAX_ANGLE
	;

	Scale_t pwmScale = __getPWMScale(servoConfig.timerConfig);

	float convertedToPWM = __convertScales(angleScale, pwmScale,
			__getCalibratedAngle(servoConfig.calibration, angle));
	setPWM(servoConfig.timerConfig.handle, servoConfig.timerConfig.channel,
			servoConfig.timerConfig.period, convertedToPWM);
}

void setServoPWMPercentage(ServoConfig_t servoConfig, float angle) {
	Scale_t angleScale;
	angleScale.min = SERVO_MIN_PERCENT;
	angleScale.max = SERVO_MAX_PERCENT;

	Scale_t pwmScale = __getPWMScale(servoConfig.timerConfig);

	float convertedToPWM = __convertScales(angleScale, pwmScale,
			__getCalibratedAngle(servoConfig.calibration, angle));
	setPWM(servoConfig.timerConfig.handle, servoConfig.timerConfig.channel,
			servoConfig.timerConfig.period, convertedToPWM);
}

Scale_t __getPWMScale(TimerConfig_t timerConfig) {
	Scale_t pwmScale;
	pwmScale.min = timerConfig.minDutyCyclePercentage * timerConfig.period;
	pwmScale.max = timerConfig.maxDutyCyclePercentage * timerConfig.period;

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
