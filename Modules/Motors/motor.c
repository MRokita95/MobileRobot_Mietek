#include "motor.h"
#include <stdbool.h>



// Private prototypes
static void setPWM(speed_t* speed, uint16_t set_speed);
static void setDirection(direction_t* dir, bool forward);
static void encoder_update_speed(encoder_t *encoder);

/**
 * @brief set pwm
 * 
 * @param speed 
 * @param set_speed 
 */
static void setPWM(speed_t* speed, uint16_t set_speed) {

    if (set_speed >= speed->tim_handle->Instance->ARR){
        set_speed = speed->tim_handle->Instance->ARR;
    }

    __HAL_TIM_SET_COMPARE(speed->tim_handle, speed->tim_channel, set_speed);
    speed->set_speed = set_speed;
}

/**
 * @brief Set the Direction of the motor with HAL 
 * 
 * @param dir 
 * @param forward
 */
static void setDirection(direction_t* dir, bool forward) {
    if (forward){
        HAL_GPIO_WritePin(dir->dir_gpio_port, dir->dir_gpio_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(dir->dir_gpio_port, dir->dir_gpio_pin, GPIO_PIN_RESET);
    }
    dir->set_forward = forward;
}



/**
 * @brief 
 * 
 * @param encoder 
 */
static void encoder_update_speed(encoder_t *encoder)
{
	encoder->pulse_count = (int16_t)__HAL_TIM_GET_COUNTER(encoder->tim_handle);
	__HAL_TIM_SET_COUNTER(encoder->tim_handle, 0);

	encoder->act_speed = (encoder->pulse_count * TIMER_FREQENCY * SECOND_IN_MINUTE) / encoder->resolution;
}

/**
 * @brief 
 * 
 * @param motor 
 */
static void control_motor_pid(motor_handle_t* motor){

    bool control_off = false;
    pid_input_t setpoint =  abs(motor->speed_setpoint);
    if (motor->speed_setpoint > 0){
        setDirection(&motor->direction, true);
    } else if (motor->speed_setpoint < 0){
        setDirection(&motor->direction, false);
    } else {
        control_off = true;
    }

    if (!control_off){
        pid_output_t pwm_output = PID_Loop(motor->pid_handle, motor->encoder.act_speed, setpoint);
        setPWM(&motor->speed, pwm_output);
        
    } else {
        setPWM(&motor->speed, 0);
    }

}


//Public interface

void motor_init(motor_handle_t* motor, bool encoder, reference_motor_t* ref) {
    motor->speed_setpoint = 0;
    motor->encoder_present = encoder;

    setDirection(&motor->direction, true);
    setPWM(&motor->speed, 0);
    HAL_TIM_PWM_Start(motor->speed.tim_handle, motor->speed.tim_channel);

    if (motor->encoder_present) {
        motor->pid_handle = PID_Init(&motor->pid_params);
        __HAL_TIM_SET_COUNTER(motor->encoder.tim_handle, 0);
        motor->encoder.pulse_count = 0;
        motor->encoder.resolution = FRONT_MOTOR_GEAR_RATIO * ENCODER_RESOLUTION * TIMER_CONF_BOTH_EDGE_T1T2;
    } 
    else if (ref != NULL){
        motor->reference.speed = ref->speed;
        motor->reference.forward = ref->forward;
    }
}

void motor_update_speed(motor_handle_t* motor) {

    encoder_update_speed(&motor->encoder);
}

void motor_set_speed(motor_handle_t* motor, int32_t speed) {

    if (!motor->encoder_present){
        return;
    }

    PID_Reset(motor->pid_handle, true);

    motor->speed_setpoint = speed;
}

void motor_task(motor_handle_t* motor){
    
    if (motor->encoder_present){    
        control_motor_pid(motor);
    } else {
        setDirection(&motor->direction, *motor->reference.forward);
        setPWM(&motor->speed, *motor->reference.speed);
    }
}

void motor_stop(motor_handle_t* motor){

    if (!motor->encoder_present){
        return;
    }

    PID_Reset(motor->pid_handle, false);

    motor->speed_setpoint = 0;
}
