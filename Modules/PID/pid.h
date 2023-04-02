/*
 * pid.h
 *
 *  Created on: Oct 11, 2022
 *      Author: micha
 */

#ifndef APP_PID_PID_H_
#define APP_PID_PID_H_


typedef struct{
	// gains
	float K;
	float KI;
	float KD;

	// low-pass filter time constans
	const float tau;

	//integrator limits
	float outMin;
	float outMax;


	uint32_t sample_time;

	//controller memory
	float prev_error;
	float integrator;
	float prev_measurement;
	float differentiator;

	float out;

} PID_controller_t;

void PID_Init(PID_controller_t *pid);

void PID_Loop(PID_controller_t *pid, float setpoint, float measurement);


#endif /* APP_PID_PID_H_ */
