#include "robot.h"
#include "main.h"

#define BIND_CONST -2


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6; 

motor_handle_t motors[MOTORS_CNT] = {
    {
        //TODO: add direction pins defs
        .speed.tim_channel = TIM2_PWM_CH1_GPIO_Port,
        .speed.tim_handle = &htim2,

        .encoder_present = true,
        .encoder.tim_handle = &htim3,

        .pid_params = {
            .max_output = 100,
            .KP = 1.0,
            .KI = 0.1,
            .KD = 0.05,
            .integratorLimit = 40,
        },
    },

    {
        //TODO: add direction pins defs
        .speed.tim_channel = TIM2_PWM_CH2_GPIO_Port,
        .speed.tim_handle = &htim2,

        .encoder_present = true,
        .encoder.tim_handle = &htim4,

        .pid_params = {
            .max_output = 100,
            .KP = 1.0,
            .KI = 0.1,
            .KD = 0.05,
            .integratorLimit = 40,
        },
    },

    {
        //TODO: add direction pins defs
        .speed.tim_channel = TIM2_PWM_CH3_GPIO_Port,
        .speed.tim_handle = &htim2,

        .encoder_present = false,
    },

    {
        //TODO: add direction pins defs
        .speed.tim_channel = TIM2_PWM_CH4_GPIO_Port,
        .speed.tim_handle = &htim2,

        .encoder_present = false,
    },
};


static reference_motor_t ref_table[4];

void Robot_Init(Mobile_Platform_t* robot){

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = &motors[idx];

        if (mot_handle->encoder_present){
            motor_init(mot_handle, true, NULL);
        } 
        else {
            ref_table[idx].forward = &motors[idx+BIND_CONST].direction.set_forward;
            ref_table[idx].speed = &motors[idx+BIND_CONST].speed.set_speed;

            motor_init(mot_handle, false, &ref_table[idx]);
        }

        robot->motors[idx] = mot_handle;
    }
}

void Robot_GetSpeed(Mobile_Platform_t* robot){

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = &robot->motors[idx];

        if (mot_handle->encoder_present){
            motor_update_speed(mot_handle);
        } 
    }
}


void Robot_SetPath(Mobile_Platform_t* robot, int32_t speed, float angle) {

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = &robot->motors[idx];

        motor_set_speed(mot_handle, speed);
    }
}

void Robot_Task(Mobile_Platform_t* robot) {

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = &robot->motors[idx];

        motor_task(mot_handle);
    }
}
