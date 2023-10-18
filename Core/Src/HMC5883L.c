/*
 * HMC5883L.c
 *
 *  Created on: Oct 17, 2023
 *      Author: Gustavo da Silva Gomes
 */

#include "HMC5883L.h"

HAL_StatusTypeDef hmc5883l_init(HMC5883LConfig_t config) {
	uint8_t registerA = CRA7;
	registerA <<= 2;
	registerA |= config.samplesNum;
	registerA <<= 3;
	registerA |= config.dataOutputRate;
	registerA <<= 2;
	registerA |= config.measurementMode;

	uint8_t registerB = config.gain;
	registerB <<= 5;

	uint8_t registerMode = 0x00;
	registerMode |= config.operatingMode;

	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Write(config.handle, HMC5883L_WRITE_ADDR, REGISTER_A_ADDR, sizeof(uint8_t), &registerA, sizeof(registerA), HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	status = HAL_I2C_Mem_Write(config.handle, HMC5883L_WRITE_ADDR, REGISTER_B_ADDR, sizeof(uint8_t), &registerB, sizeof(registerB), HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	status = HAL_I2C_Mem_Write(config.handle, HMC5883L_WRITE_ADDR, REGISTER_MODE_ADDR, sizeof(uint8_t), &registerMode, sizeof(registerMode), HAL_MAX_DELAY);
	return status;
}

HAL_StatusTypeDef hmc5883l_read(HMC5883LConfig_t config, HMC5883LData_t *data) {
	HAL_StatusTypeDef status;

	uint8_t addrsLow[3] = {DOX_L_ADDR, DOY_L_ADDR, DOZ_L_ADDR};
	uint8_t addrsHigh[3] = {DOX_H_ADDR, DOY_H_ADDR, DOZ_H_ADDR};
	uint8_t axisArr[3] = {__X,__Y,__Z};

	uint8_t dataHigh = 0, dataLow = 0;
	int16_t axisData = 0;

	for(int i=0; i<3; i++) {
		// msb
		status = HAL_I2C_Mem_Read(config.handle, HMC5883L_READ_ADDR, addrsHigh[i], sizeof(uint8_t), &dataHigh, sizeof(uint8_t), HAL_MAX_DELAY);
		if(status != HAL_OK) return status;
		// lsb
		status = HAL_I2C_Mem_Read(config.handle, HMC5883L_READ_ADDR, addrsLow[i], sizeof(uint8_t), &dataLow, sizeof(uint8_t), HAL_MAX_DELAY);
		if(status != HAL_OK) return status;

		axisData = dataHigh;
		axisData <<= 8;
		axisData |= dataLow;

		status = __setData(data, axisArr[i], axisData);
		if(status != HAL_OK) return status;
	}

	return HAL_OK;
}

HAL_StatusTypeDef __setData(HMC5883LData_t *data, uint8_t axis, int16_t axisData) {
	switch(axis) {
		case __X:
			data->x = axisData;
			return HAL_OK;
		case __Y:
			data->y = axisData;
			return HAL_OK;
		case __Z:
			data->z = axisData;
			return HAL_OK;
		default:
			return HAL_ERROR;
	}
}
