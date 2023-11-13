/*
 * PID.c
 *
 *  Created on: Nov 13, 2023
 *      Author: gusta
 */

#include <math.h>
#include "PID.h"

void PID_Create(PID_Controller_t *controller, float kp, float ki, float kd, int periodMs) {
	controller->Kp = kp;
	controller->Ki = ki;
	controller->Kd = kd;

	controller->setpoint = 0;
	controller->measured = 0;

	controller->errorAcc = 0;
	controller->errorDer = 0;

	controller->errorArr[PID_CURRENT] = 0;
	controller->errorArr[PID_LAST] = 0;

	controller->periodMs = periodMs;

	// no saturation limits by default
	controller->maxOutput = INFINITY;
	controller->minOutput = -INFINITY;
}

void PID_SetSaturationLimits(PID_Controller_t *controller, float min, float max) {
	controller->minOutput = min;
	controller->maxOutput = max;
}

void PID_SetSetpoint(PID_Controller_t *controller, float setpoint) {
	controller->setpoint = setpoint;
}

void PID_ProcessInput(PID_Controller_t *controller, float input) {
	float error = setpoint-input;

	controller->errorAcc += error*controller->periodMs;

	controller->errorArr[PID_LAST] = controller->errorArr[PID_CURRENT];
	controller->errorArr[PID_CURRENT] = error;

	controller->errorDer = (controller->error[PID_CURRENT] - controller->errorArr[PID_LAST])/controller->periodMS;
}

float PID_CalculateControlAction(PID_Controller_t *controller) {
	float P = controller->Kp*errorArr[PID_CURRENT];
	float I = controller->Ki*errorAcc;
	float D = controller->Kd*errorDer;

	float PID = __PID_SaturateOutput(controller, P+I+D);

	return PID;
}

float __PID_SaturateOutput(PID_Controller_t *controller, float originalOutput) {
	// limit integral error
	if(controller->errorAcc > controller->maxOutput) controller->errorAcc = controller->maxOutput;

	if(originalOutput > controller->maxOutput) return controller->maxOutput;
	if(originalOutput < controller->minOutput) return controller->minOutput;
	return originalOutput;
}
