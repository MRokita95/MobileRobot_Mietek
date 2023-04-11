#include "robot.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

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
    robot->distance.mode_on = false;
    robot->distance.setpoint_distance = 0;
    robot->distance.actual_distance = 0;
    robot->distance.start_time = 0u;

    robot->timer.mode_on = false;
    robot->distance.actual_distance = 0;
    robot->distance.start_time = 0u;


    robot->initialized = true;
}

void Robot_UpdateMotionStatus(Mobile_Platform_t* robot){

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = &robot->motors[idx];

        if (mot_handle->encoder_present){
            motor_update_speed(mot_handle);
        } 
    }
    Robot_CalcDistance(robot);
}


void Robot_SetPath(Mobile_Platform_t* robot, int32_t speed, float angle) {

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = &robot->motors[idx];

        motor_set_speed(mot_handle, speed);
    }
}

void Robot_SetDistance(Mobile_Platform_t* robot, int32_t distance) {

    robot->distance.mode_on = true;
    robot->distance.actual_distance = 0;
    robot->distance.setpoint_distance = distance;
    robot->distance.start_time = HAL_GetTick()/portTICK_PERIOD_MS;
    robot->distance.current_time = robot->distance.start_time;
}

void Robot_CalcDistance(Mobile_Platform_t* robot) {
    
    if (!robot->distance.mode_on){
        return;
    }

    int32_t actual_dist = 0;
    int32_t encoders = 0;
    uint32_t dt = (uint32_t)(HAL_GetTick()/portTICK_PERIOD_MS) - (robot->distance.current_time);

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = &robot->motors[idx];

        if (mot_handle->encoder_present){
            actual_dist = mot_handle->encoder.act_speed * dt;
            encoders++;
        } 
    }
    robot->distance.actual_distance = robot->distance.actual_distance + (actual_dist / encoders);
    robot->distance.current_time += dt;
}

void Robot_StartTimer(Mobile_Platform_t* robot, uint32_t ms)
{
    robot->timer.mode_on = true;
    robot->timer.start_time = (uint32_t)(HAL_GetTick()/portTICK_PERIOD_MS);
    robot->timer.stop_time = ms;
}

void Robot_Task(Mobile_Platform_t* robot) {

    if (robot->distance.mode_on){
        if (robot->distance.actual_distance >= robot->distance.setpoint_distance){
            Robot_Stop(robot);
        }
    }

    if (robot->timer.mode_on){
        uint32_t dt = (uint32_t)(HAL_GetTick()/portTICK_PERIOD_MS);
        if (robot->timer.start_time + dt >= robot->timer.stop_time){
            Robot_Stop(robot);
        }
    }

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = &robot->motors[idx];

        motor_task(mot_handle);
    }
}

void Robot_Stop(Mobile_Platform_t* robot) {

    robot->distance.mode_on = false;
    robot->distance.actual_distance = 0;
    robot->distance.setpoint_distance = 0;
    robot->distance.start_time = 0;
    robot->distance.current_time = 0;

    robot->timer.mode_on = false;
    robot->timer.start_time = 0;
    robot->timer.stop_time = 0;

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = &robot->motors[idx];

        motor_stop(mot_handle);
    }
}
