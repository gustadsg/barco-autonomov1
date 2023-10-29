/*
 * DCMotor.c
 *
 *  Created on: Oct 11, 2023
 *      Author: João
 */

#include "DCMotor.h"

void DCMOTOR_SetSpeedPercentage(DCMOTOR_TimerConfig_t timerConfig, float percentage) {
	int minPercent = DC_MOTOR_MIN_PERCENT;
	int maxPulseLength = timerConfig.period;
	uint16_t pulseLength = percentage * timerConfig.period;
	if (pulseLength < minPercent) pulseLength = 0;
	if (pulseLength > maxPulseLength) pulseLength = maxPulseLength;
	setPWM(timerConfig.handle, timerConfig.channel, timerConfig.period, pulseLength);
}
