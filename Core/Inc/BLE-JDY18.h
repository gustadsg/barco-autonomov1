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
#include "stm32f4xx_hal.h"

#define JDY18_BUFFER_SZ 1024
#define JDY18_MAX_DEVICES 5

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

typedef struct {
	char name[18];
	int id;
	int rssi;
} JDY18_Device_t;


void JDY18_Setup(UART_HandleTypeDef* bleHuart);
void JDY18_SetConfig(char* cfg);
void JDY18_Reset();
void JDY18_SendCommand(char* commandPrefix, char* commandParam);
void JDY18_SetName(char* name);
void JDY18_GetName(char* name);
void JDY18_SetRole(JDY18_Role_t role);
void JDY18_SetBaudRate(JDY18_BaudRate_t baudRate);
void JDY18_Scan(JDY18_Device_t* devices);
void __JDY18_GetDevicesFromScanStr(char* str, JDY18_Device_t* devices, int8_t maxDevices);

void __JDY18_ReceiveStr(char* str);

#endif /* INC_BLE_JDY18_H_ */
