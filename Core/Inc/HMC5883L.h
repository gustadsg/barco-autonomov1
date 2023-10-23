/*
 * HMC588L.h
 *
 *  Created on: Oct 17, 2023
 *      Author: Gustavo da Silva Gomes
 */

#ifndef INC_HMC5883L_H_
#define INC_HMC5883L_H_

#include "inttypes.h"
#include "math.h"
#include "stm32f4xx_hal.h"

#define CRA7 0x0

#define CRB4 0x0
#define CRB3 0x0
#define CRB2 0x0
#define CRB1 0x0
#define CRB0 0x0

#define MR7 0X0
#define MR6 0X0
#define MR5 0X0
#define MR4 0X0
#define MR3 0X0
#define MR2 0X0

#define REGISTER_A_ADDR 0x00
#define REGISTER_B_ADDR 0x01
#define REGISTER_MODE_ADDR 0x02

#define DOX_H_ADDR 0x03
#define DOX_L_ADDR 0x04

#define DOY_H_ADDR 0x05
#define DOY_L_ADDR 0x06

#define DOZ_H_ADDR 0x07
#define DOZ_L_ADDR 0x08

#define STATUS_REGISTER_ADDR 0x09

#define IDENTIFICATION_REGISTER_A_ADDR 0x0A
#define IDENTIFICATION_REGISTER_B_ADDR 0x0B
#define IDENTIFICATION_REGISTER_C_ADDR 0x0C

#define HMC5883L_READ_ADDR 0x1E << 1
#define HMC5883L_WRITE_ADDR 0x1E << 1

#define HMC5883L_BYTE_SZ 1

#define __X 0
#define __Y 1
#define __Z 2

typedef enum {
	SAMPLES_1,
	SAMPLES_2,
	SAMPLES_4,
	SAMPLES_8
} SamplesNum_t;

typedef enum {
	DOR_0_75,
	DOR_1_5,
	DOR_3,
	DOR_7_5,
	DOR_15,
	DOR_30,
	DOR_75
} DataOutputRate_t;

typedef enum {
	MESUAREMENT_NORMAL,
	MESUAREMENT_POSITIVE,
	MESUAREMENT_NEGATIVE
} MesuarementMode_t;

typedef enum {
	GAIN_0_88,
	GAIN_1_3,
	GAIN_1_9,
	GAIN_2_5,
	GAIN_4_0,
	GAIN_4_7,
	GAIN_5_6,
	GAIN_8_1
} Gain_t;

typedef enum {
	CONTINUOS_MODE,
	SINGLE_MODE,
	IDLE_MODE,
	IDLE_MODE2
} OperatingMode_t;

typedef struct {
	I2C_HandleTypeDef* handle;
	SamplesNum_t samplesNum;
	DataOutputRate_t dataOutputRate;
	MesuarementMode_t measurementMode;
	Gain_t gain;
	OperatingMode_t operatingMode;
} HMC5883LConfig_t;

typedef struct {
	uint16_t x;
	uint16_t y;
	uint16_t z;
	float degrees;
	float radians;
} HMC5883LData_t;

typedef enum {
	READY = 1,
	LOCKED
} Status_t;

HAL_StatusTypeDef hmc5883l_init(HMC5883LConfig_t config);
HAL_StatusTypeDef hmc5883l_read(HMC5883LConfig_t config, HMC5883LData_t *data);
HAL_StatusTypeDef __setDataAxis(HMC5883LData_t *data, uint8_t axis, int16_t axisData);
HAL_StatusTypeDef __getStatus(HMC5883LConfig_t config, Status_t *status);
void __setDataAngles(HMC5883LData_t *data);

#endif /* INC_HMC5883L_H_ */
