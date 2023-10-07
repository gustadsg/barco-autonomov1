/*
 * BLE-JDY18.c
 *
 *  Created on: Sep 19, 2023
 *      Author: gusta
 */
#include "BLE-JDY18.h"

UART_HandleTypeDef* husart;

void setup(UART_HandleTypeDef* handle) {
	husart = handle;
}

void setConfig(char* config) {
	sendCommand("PERM", config);
}

void reset() {
	sendCommand("DEFAULT", "");
}

void sendCommand(char* commandPrefix, char* commandParam) {
	char* command = "AT+";

	int commandSize = strlen(command) + strlen(commandPrefix) + strlen(commandParam) + 3;
	command = malloc(commandSize);
	memset(command, 0, commandSize);

	strcat(command, "AT+");
	strcat(command, commandPrefix);
	strcat(command, commandParam);
	strcat(command, "\r\n");

	HAL_UART_Transmit(husart, (uint8_t*) command, commandSize, HAL_MAX_DELAY);
}

void setName(char* name) {
	char command[30] = "AT+";
	strcat(command, name);
	sendCommand("NAME", name);
}

void getName(char* name) {
	name = malloc(24);
	memset(name, 0, 24);

	sendCommand("NAME", "");

	HAL_UART_Receive(husart, (uint8_t*) name, 24, HAL_MAX_DELAY);
}

void setRole(Role_t role) {
	char roleStr[2];
	itoa(role, roleStr, 10);

	sendCommand("ROLE", roleStr);
}

void setBaudRate(BaudRate_t baud) {
	char baudStr[5] = "";
	itoa(baud, baudStr, 10);


	sendCommand("BAUD", baudStr);
}

void scan(Device_t *devices) {
	char data[BUFFER_SZ];
	memset(data, 0, BUFFER_SZ);

	sendCommand("IQN", "");
	HAL_UART_Receive(husart, (uint8_t*) data, BUFFER_SZ, 1000);

	_getDevicesFromScanStr(data, devices, MAX_DEVICES);
}

void _getDevicesFromScanStr(char* str, Device_t *devices, int8_t maxDevices) {
	char inputCopy[BUFFER_SZ];
	strcpy(inputCopy, str);

	const char *token = strtok(inputCopy, "\r\n");
	int deviceCount = 0;

	while (token != NULL) {
		int lineHasDeviceInfo = strstr(token, "+DEV:") != NULL;
		if (lineHasDeviceInfo) {
			int id, rssi;
			char name[18];

			char* deviceInfoRegex = "+DEV:%d=%*[^,],%d,%18[^\\r\\n]";

			if (sscanf(token, deviceInfoRegex, &id, &rssi, name) == 3) {
				if (deviceCount < maxDevices) {
					devices[deviceCount].id = id;
					devices[deviceCount].rssi = rssi;

					memset(devices[deviceCount].name, 0, 18);
					strcpy(devices[deviceCount].name, name);
					deviceCount++;
				}
			}
		}
		token = strtok(NULL, "\r\n");
	}
}
