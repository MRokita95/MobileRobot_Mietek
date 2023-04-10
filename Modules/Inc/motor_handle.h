#ifndef MOTOR_HANDLE_H_
#define MOTOR_HANDLE_H_

#include <stdbool.h>
#include "pid_handle.h"
#include "stm32f4xx_hal.h"


#define MOTORS_CNT  4

#define ENCODER_RESOLUTION 8
#define TIMER_CONF_BOTH_EDGE_T1T2	        4
#define FRONT_MOTOR_GEAR_RATIO 120
#define REAR_MOTOR_GEAR_RATIO 120

#define	TIMER_FREQENCY				10
#define	SECOND_IN_MINUTE			60

typedef struct {
    bool set_forward;
    GPIO_TypeDef* dir_gpio_port;
    uint16_t dir_gpio_pin;
} direction_t;

typedef struct {
    uint16_t set_speed;
    uint8_t tim_channel;
    TIM_HandleTypeDef *tim_handle;
} speed_t;

typedef struct {
    int32_t act_speed;
    uint8_t resolution;
    int32_t pulse_count;
    TIM_HandleTypeDef *tim_handle;	
} encoder_t;

typedef struct{
    pid_output_t* speed;
    bool* forward;
} reference_motor_t;

typedef struct{
    
    speed_t speed;
    direction_t direction;
    bool encoder_present;
    encoder_t encoder;

    int32_t speed_setpoint;
    const PID_parameters_t pid_params;
    PID_handle_t pid_handle;

    reference_motor_t reference;
} motor_handle_t;



void motor_init(motor_handle_t* motor, bool encoder, reference_motor_t* reference_mot_speed);

void motor_update_speed(motor_handle_t* motor);

void motor_set_speed(motor_handle_t* motor, int32_t speed);

void motor_task(motor_handle_t* motor);

#endif