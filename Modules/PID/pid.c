/*
 * pid.c
 *
 *  Created on: Oct 11, 2022
 *      Author: micha
 */

#include "pid.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

PID_handle_t PID_Init(const PID_parameters_t* parameters)
{
    PID_handle_t pid = malloc(sizeof(struct PID_controller_t));
    if (pid == NULL){
        return NULL;
    }
    
    pid->params = parameters;

    pid->integrator = 0;
    pid->out = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->tau = 0u;
    pid->sample_time = 0u;
    pid->prev_error = 0;

    pid->last_tick = HAL_GetTick();

    return pid;
}

pid_output_t PID_Loop(PID_handle_t pid_instance, pid_input_t input, pid_input_t setpoint)
{
    int32_t error = (int32_t)setpoint - (int32_t)input;

    pid_instance->integrator += error;

    float p_calc = pid_instance->params->KP * (float)error;

    //simpler version
    //float i_calc = pid_instance->params->KI * (float)pid_instance->integrator;

    // could be changed to Time_Delta :)
    uint32_t current_tick = HAL_GetTick();
    uint32_t delta_ms = current_tick/portTICK_PERIOD_MS - pid_instance->last_tick/portTICK_PERIOD_MS;
    float delta_s = (float)delta_ms / 1000.f;
    float i_calc = (float)pid_instance->integrator + pid_instance->params->KI * ((float)error * delta_s);
    
    //simpler version
    //float d_calc = pid_instance->params->KD * (float)(error - pid_instance->prev_error);

    float d_calc = pid_instance->params->KD * (float)(error - pid_instance->prev_error)/delta_s;

    //anti wind-up
    if (i_calc > pid_instance->params->integratorLimit){
        i_calc = pid_instance->params->integratorLimit;
    } else if (i_calc < -pid_instance->params->integratorLimit){
        i_calc = -pid_instance->params->integratorLimit;
    }


    pid_instance->prev_error = error;
    pid_instance->out = p_calc + i_calc + d_calc;
    pid_instance->last_tick = current_tick;

    if (pid_instance->out > pid_instance->params->max_output){
        pid_instance->out = pid_instance->params->max_output;
    }

    return (pid_output_t)pid_instance->out;
} 

void PID_Reset(PID_handle_t pid)
{
    pid->integrator = 0;
    pid->out = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->tau = 0u;
    pid->sample_time = 0u;
    pid->prev_error = 0;
    pid->last_tick = HAL_GetTick();
}
