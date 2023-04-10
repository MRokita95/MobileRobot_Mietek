#ifndef PID_HANDLE_H_
#define PID_HANDLE_H_

#include <stdlib.h>
#include <stdint.h>

typedef uint16_t pid_output_t; 
typedef uint16_t pid_input_t; 

typedef struct PID_controller_t* PID_handle_t;

typedef struct{
    float KP;
    float KI;
    float KD;
    float integratorLimit;
    pid_output_t max_output;
} PID_parameters_t;


//Public functions


PID_handle_t PID_Init(const PID_parameters_t* parameters);

pid_output_t PID_Loop(PID_handle_t pid_instance, pid_input_t input, pid_input_t setpoint);

void PID_Reset(PID_handle_t pid_instance);

#endif
