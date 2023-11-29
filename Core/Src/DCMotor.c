/*
 * DCMotor.c
 *
 *  Created on: Oct 11, 2023
 *      Author: João
 */

#include "DCMotor.h"

void DCMOTOR_SetSpeedPercentage(DCMOTOR_Config_t config, float percentage) {
	DCMOTOR_TimerConfig_t timerConfig = config.timerConfig;

	int minPercent = DC_MOTOR_MIN_PERCENT;
	int maxPulseLength = timerConfig.period;
	uint16_t pulseLength = percentage * timerConfig.period;
	if (pulseLength < minPercent) pulseLength = 0;
	if (pulseLength > maxPulseLength) pulseLength = maxPulseLength;
	PWM_SetValue(timerConfig.handle, timerConfig.channel, timerConfig.period, pulseLength);
}


void DCMOTOR_SetDirection(DCMOTOR_Config_t config, DCMOTOR_Direction_t direction) {
	GPIO_PinState data = GPIO_PIN_RESET;

	HAL_GPIO_WritePin(config.data.GPIOx, config.data.GPIO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(config.latch.GPIOx, config.latch.GPIO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(config.latch.GPIOx, config.clk.GPIO_Pin, GPIO_PIN_RESET);

	unsigned char comparator = 0x80;

	for (int i = 0; i<8; i++){
		if (direction & comparator){
		  data = GPIO_PIN_SET;
		}
		else{
		  data = GPIO_PIN_RESET;
		}

		comparator = comparator >> 1;

		HAL_GPIO_WritePin(config.data.GPIOx, config.data.GPIO_Pin, data);
		HAL_Delay(1);
		HAL_GPIO_WritePin(config.latch.GPIOx, config.clk.GPIO_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(config.latch.GPIOx, config.clk.GPIO_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
	}

	HAL_GPIO_WritePin(config.latch.GPIOx, config.latch.GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(config.latch.GPIOx, config.latch.GPIO_Pin, GPIO_PIN_RESET);
}
