/*
 * BLE-JDY18.h
 *
 *  Created on: Sep 18, 2023
 *      Author: gusta
 */

#ifndef INC_BLE_JDY18_H_
#define INC_BLE_JDY18_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "stm32f4xx_hal.h"

#define JDY18_BUFFER_SZ 1024
#define JDY18_MAX_DEVICES 5

#define JDY18_DEFAULT_POWER -69
#define JDY18_N 2

typedef enum {
	JDY18_ROLE_SLAVE = 0,
	JDY18_ROLE_MASTER=1,
	JDY18_ROLE_BEACON=3,
} JDY18_Role_t;

typedef enum {
	JDY18_Baud_1200=1,
	JDY18_Baud_2400,
	JDY18_Baud_4800,
	JDY18_Baud_9600,
	JDY18_Baud_19200,
	JDY18_Baud_38400,
	JDY18_Baud_57600,
	JDY18_Baud_115200,
	JDY18_Baud_230400,
} JDY18_BaudRate_t;

typedef enum {
	// @brief Sets permissions configuration of the device
	JDY18_Command_SetConfig=0,
	// @brief Reset device to factory settings
	JDY18_Command_Reset,
	// @brief Sets the name of the device
	JDY18_Command_SetName,
	JDY18_Command_GetName,
	JDY18_Command_SetRole,
	JDY18_Command_SetBaudRate,
	JDY18_Command_Scan,
} JDY18_CommandPrefix_t;

typedef struct {
	char name[18];
	int id;
	int rssi;
	float distance;
} JDY18_Device_t;

/**
 * @brief Sets internal reference to UART handle
 * @param Pointer to the UART handle
 */
void JDY18_Setup(UART_HandleTypeDef* bleHuart);

/**
 * @brief Sends string command to device
 * @param The command to be sent
 * @param The data to be set. If no data is being set, this param should be ""
 */
void JDY18_SendCommand(JDY18_CommandPrefix_t commandIndex, char* commandParam);

/**
 * @brief Gets the current name of the device
 * @param variable to store the name
 */
void JDY18_GetName(char* name);

/**
 * @brief Sets the device's role
 * @param Role to be set
 */
void JDY18_SetRole(JDY18_Role_t role);

/**
 * @brief Sets the device's baud rate
 * @param The desired baud rate
 */
void JDY18_SetBaudRate(JDY18_BaudRate_t baudRate);

/**
 * @briefs Scans for bluetooth devices nearby
 * @param Pointer to devices list
 */
void JDY18_Scan(JDY18_Device_t* devices);

/**
 * @brief Calculates the distance of a list of devices based on its RSSI values
 * @param Pointer to list of devices
 * @param Number of devices in the list
 */
void __JDY18_GetDistanceFromRssi(JDY18_Device_t *devices, int8_t numDevices);

/**
 * @brief Transforms a scan string into a list of bluetooth devices
 * @param The scan string
 * @param A pointer to the list of devices
 * @param Num of maximum devices to pu into the list
 */
int __JDY18_GetDevicesFromScanStr(char* str, JDY18_Device_t* devices, int8_t maxDevices);


#endif /* INC_BLE_JDY18_H_ */
