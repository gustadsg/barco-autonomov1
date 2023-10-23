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

	status = HAL_I2C_Mem_Write(config.handle, HMC5883L_WRITE_ADDR, REGISTER_A_ADDR, I2C_MEMADD_SIZE_8BIT, &registerA, HMC5883L_BYTE_SZ, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	status = HAL_I2C_Mem_Write(config.handle, HMC5883L_WRITE_ADDR, REGISTER_B_ADDR, I2C_MEMADD_SIZE_8BIT, &registerB, HMC5883L_BYTE_SZ, HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	status = HAL_I2C_Mem_Write(config.handle, HMC5883L_WRITE_ADDR, REGISTER_MODE_ADDR, I2C_MEMADD_SIZE_8BIT, &registerMode, HMC5883L_BYTE_SZ, HAL_MAX_DELAY);
	return status;
}

HAL_StatusTypeDef hmc5883l_read(HMC5883LConfig_t config, HMC5883LData_t *data) {
	HAL_StatusTypeDef status;
	Status_t readyOrLocked = LOCKED;

	// wait until data registers are unlocked
	while(readyOrLocked == LOCKED) {
		status = __getStatus(config, &readyOrLocked);
		if(status != HAL_OK) return status;
		if(readyOrLocked == LOCKED) HAL_Delay(10);
	}

	uint8_t addrsLow[3] = {DOX_L_ADDR, DOY_L_ADDR, DOZ_L_ADDR};
	uint8_t addrsHigh[3] = {DOX_H_ADDR, DOY_H_ADDR, DOZ_H_ADDR};
	uint8_t axisArr[3] = {__X,__Y,__Z};

	uint8_t dataHigh = 0, dataLow = 0;
	int16_t axisData = 0;

	for(int i=0; i<3; i++) {
		// get most significant bits
		status = HAL_I2C_Mem_Read(config.handle, HMC5883L_READ_ADDR, addrsHigh[i], I2C_MEMADD_SIZE_8BIT, &dataHigh, HMC5883L_BYTE_SZ, HAL_MAX_DELAY);
		if(status != HAL_OK) return status;
		// get less significant bits
		status = HAL_I2C_Mem_Read(config.handle, HMC5883L_READ_ADDR, addrsLow[i], I2C_MEMADD_SIZE_8BIT, &dataLow, HMC5883L_BYTE_SZ, HAL_MAX_DELAY);
		if(status != HAL_OK) return status;

		axisData = dataHigh;
		axisData <<= 8;
		axisData |= dataLow;

		status = __setDataAxis(data, axisArr[i], axisData);
		if(status != HAL_OK) return status;
	}

	__setDataAngles(data);

	return HAL_OK;
}

HAL_StatusTypeDef __setDataAxis(HMC5883LData_t *data, uint8_t axis, int16_t axisData) {
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

void __setDataAngles(HMC5883LData_t *data) {
	data->radians = atan2f(data->x, data->y);
	data->degrees = data->radians * (180.0/M_PI);
}

HAL_StatusTypeDef __getStatus(HMC5883LConfig_t config, Status_t *registerStatus) {
	HAL_StatusTypeDef status;
	uint8_t statusRegisterData = 0;

	status = HAL_I2C_Mem_Read(config.handle, HMC5883L_READ_ADDR, STATUS_REGISTER_ADDR, sizeof(uint8_t), &statusRegisterData, sizeof(uint8_t), HAL_MAX_DELAY);
	if(status != HAL_OK) return status;

	// clear the six most significant bits, since they are not used
	statusRegisterData &= 0b00000011;

	*registerStatus = statusRegisterData;

	// the only possible values read are READY and LOCKED, otherwise an error occurred in the communication
	return (statusRegisterData == READY || statusRegisterData == LOCKED) ? HAL_OK : HAL_ERROR;
}
