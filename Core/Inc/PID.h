/*
 * PID.h
 *
 *  Created on: Nov 13, 2023
 *      Author: gusta
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define PID_CURRENT 0
#define PID_LAST 1

typedef struct {
	float Kp;
	float Ki;
	float Kd;

	float setpoint;
	float measured;

	float maxOutput;
	float minOutput;

	float errorArr[2];
	float errorAcc;
	float errorDer;

	int periodMs;
} PID_Controller_t;

void PID_Create(PID_Controller_t *controller, float kp, float ki, float kd, int periodMs);

void PID_SetSaturationLimits(PID_Controller_t *controller, float min, float max);

void PID_SetSetpoint(PID_Controller_t *controller, float setpoint);

void PID_ProcessInput(PID_Controller_t *controller, float input);

float PID_CalculateControlAction(PID_Controller_t *controller);

float __PID_SaturateOutput(PID_Controller_t *controller, float originalOutput);



#endif /* INC_PID_H_ */
