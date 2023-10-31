/*
 * BLE-JDY18.c
 *
 *  Created on: Sep 19, 2023
 *      Author: gusta
 */
#include "BLE-JDY18.h"

UART_HandleTypeDef* husart;

void JDY18_Setup(UART_HandleTypeDef* handle) {
	husart = handle;
}

void JDY18_SetConfig(char* config) {
	JDY18_SendCommand("PERM", config);
}

void JDY18_Reset() {
	JDY18_SendCommand("DEFAULT", "");
}

void JDY18_SendCommand(char* commandPrefix, char* commandParam) {
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

void JDY18_SetName(char* name) {
	char command[30] = "AT+";
	strcat(command, name);
	JDY18_SendCommand("NAME", name);
}

void JDY18_GetName(char* name) {
	name = malloc(24);
	memset(name, 0, 24);

	JDY18_SendCommand("NAME", "");

	HAL_UART_Receive(husart, (uint8_t*) name, 24, HAL_MAX_DELAY);
}

void JDY18_SetRole(JDY18_Role_t role) {
	char roleStr[2];
	itoa(role, roleStr, 10);

	JDY18_SendCommand("ROLE", roleStr);
}

void JDY18_SetBaudRate(JDY18_BaudRate_t baud) {
	char baudStr[5] = "";
	itoa(baud, baudStr, 10);


	JDY18_SendCommand("BAUD", baudStr);
}

void JDY18_Scan(JDY18_Device_t *devices) {
	char data[JDY18_BUFFER_SZ];
	memset(data, 0, JDY18_BUFFER_SZ);

	JDY18_SendCommand("IQN", "");
	HAL_UART_Receive(husart, (uint8_t*) data, JDY18_BUFFER_SZ, 1000);

	int numDevices = __JDY18_GetDevicesFromScanStr(data, devices, JDY18_MAX_DEVICES);
	__JDY18_GetDistanceFromRssi(devices, numDevices);
}

void __JDY18_GetDistanceFromRssi(JDY18_Device_t *devices, int8_t numDevices) {
	for(int i=0; i<numDevices; i++) {
		devices[i].distance = pow(10, (JDY18_DEFAULT_POWER-devices[i].rssi)/(JDY18_N*10));
	}
}

int __JDY18_GetDevicesFromScanStr(char* str, JDY18_Device_t *devices, int8_t maxDevices) {
	char inputCopy[JDY18_BUFFER_SZ];
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

	return deviceCount;
}
