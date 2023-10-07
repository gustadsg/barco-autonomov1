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

#define BUFFER_SZ 1024
#define MAX_DEVICES 5

typedef enum {
	SLAVE = 0,
	MASTER=1,
	BEACON=3,
} Role_t;

typedef enum {
	Baud_1200=1,
	Baud_2400,
	Baud_4800,
	Baud_9600,
	Baud_19200,
	Baud_38400,
	Baud_57600,
	Baud_115200,
	Baud_230400,
} BaudRate_t;

typedef struct {
	char name[18];
	int id;
	int rssi;
} Device_t;


void setup(UART_HandleTypeDef* bleHuart);
void setConfig(char* cfg);
void reset();
void sendCommand(char* commandPrefix, char* commandParam);
void setName(char* name);
void getName(char* name);
void setRole(Role_t role);
void setBaudRate(BaudRate_t baudRate);
void scan(Device_t* devices);
void _getDevicesFromScanStr(char* str, Device_t* devices, int8_t maxDevices);

void _receiveStr(char* str);

#endif /* INC_BLE_JDY18_H_ */
