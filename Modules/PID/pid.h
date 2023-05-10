/*
 * pid.h
 *
 *  Created on: Oct 11, 2022
 *      Author: micha
 */

#ifndef APP_PID_PID_H_
#define APP_PID_PID_H_

#include "pid_handle.h"

struct PID_controller_t{
	//parameters
	PID_parameters_t* params;

	// low-pass filter time constans
	float tau;

	uint32_t sample_time;

	//controller memory
	int32_t prev_error;
	int32_t integrator;
	float prev_measurement;
	float differentiator;

	uint32_t last_tick;

	float out;

};


#endif /* APP_PID_PID_H_ */
