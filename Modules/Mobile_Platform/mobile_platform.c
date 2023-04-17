#include "robot.h"
#include "mobile_platform.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#define BIND_CONST -2


extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

motor_handle_t motors[MOTORS_CNT] = {
    {
        //TODO: add direction pins defs
        .speed.tim_channel = TIM_CHANNEL_1,
        .speed.tim_handle = &htim2,

        .direction.dir_gpio_port = GPIO_MOT1_DIR_GPIO_Port,
        .direction.dir_gpio_pin = GPIO_MOT1_DIR_Pin,

        .encoder_present = true,
        .encoder.tim_handle = &htim3,

        .pid_params = {
            .max_output = 100,
            .KP = 0.06,
            .KI = 0.9,
            .KD = 0.4,
            .integratorLimit = 40,
        },
    },

    {
        //TODO: add direction pins defs
        .speed.tim_channel = TIM_CHANNEL_2,
        .speed.tim_handle = &htim2,

        .direction.dir_gpio_port = GPIO_MOT2_DIR_GPIO_Port,
        .direction.dir_gpio_pin = GPIO_MOT2_DIR_Pin,

        .encoder_present = true,
        .encoder.tim_handle = &htim8,

        .pid_params = {
            .max_output = 100,
            .KP = 0.06,
            .KI = 0.9,
            .KD = 0.4,
            .integratorLimit = 40,
        },
    },

    {
        //TODO: add direction pins defs
        .speed.tim_channel = TIM_CHANNEL_3,
        .speed.tim_handle = &htim2,

        .direction.dir_gpio_port = GPIO_MOT3_DIR_GPIO_Port,
        .direction.dir_gpio_pin = GPIO_MOT3_DIR_Pin,

        .encoder_present = false,
    },

    {
        //TODO: add direction pins defs
        .speed.tim_channel = TIM_CHANNEL_4,
        .speed.tim_handle = &htim2,

        .direction.dir_gpio_port = GPIO_MOT4_DIR_GPIO_Port,
        .direction.dir_gpio_pin = GPIO_MOT4_DIR_Pin,

        .encoder_present = false,
    },
};

typedef struct{
    robot_status_t status;
    distance_mode_t distance;
    time_mode_t timer;
    float actual_linear_speed;
    SemaphoreHandle_t activate_mode;
    SemaphoreHandle_t evaluate_status;
} robot_internal_state_t;


static robot_internal_state_t m_rob_state;

static reference_motor_t ref_table[4];


static void eval_motion_status(Mobile_Platform_t* robot) {

    int32_t actual_dist = 0;
    float actual_linear_speed;
    float sum_speed = 0;
    int32_t encoders = 0;
    uint32_t dt = (uint32_t)(HAL_GetTick()/portTICK_PERIOD_MS) - (m_rob_state.distance.current_time);

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = robot->motors[idx];

        if (mot_handle->encoder_present){
            actual_linear_speed = (float)(mot_handle->encoder.act_speed) * PI * WHEEL_DIAMATER / SECOND_IN_MINUTE;
            sum_speed += actual_linear_speed;
            actual_dist += actual_linear_speed * dt;
            encoders++;
        } 
    }
    
    robot->actual_speed = (int32_t)(sum_speed / encoders);

    if (!m_rob_state.distance.mode_on){
        return;
    }
    m_rob_state.distance.actual_distance = m_rob_state.distance.actual_distance + (actual_dist / encoders);
    m_rob_state.distance.current_time += dt;

    robot->current_distance =  m_rob_state.distance.actual_distance;
}


static void set_rob_state(robot_status_t status){
    if( xSemaphoreTake(m_rob_state.evaluate_status, 10) == pdPASS){
        m_rob_state.status = status;
        xSemaphoreGive(m_rob_state.evaluate_status);
    }
}


/**
* PUBLIC FUNCTIONS
 */


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

    robot->speed_setpoint = 0;
    robot->actual_speed = 0;
    robot->current_distance = 0;

    m_rob_state.activate_mode = xSemaphoreCreateMutex();
    m_rob_state.evaluate_status = xSemaphoreCreateMutex();

    set_rob_state(ROB_IDLE);
}

void Robot_UpdateMotionStatus(Mobile_Platform_t* robot){

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = robot->motors[idx];

        if (mot_handle->encoder_present){
            motor_update_speed(mot_handle);
        } 
    }
    eval_motion_status(robot);
}


void Robot_SetPath(Mobile_Platform_t* robot, int32_t speed, float angle) {

    (void)angle;
    robot->speed_setpoint = speed;
    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = robot->motors[idx];

        int32_t rpm_setpoint = (int32_t)(robot->speed_setpoint * SECOND_IN_MINUTE / (int32_t)(PI * WHEEL_DIAMATER));
        motor_set_speed(mot_handle, rpm_setpoint);
    }
}

void Robot_SetDistance(Mobile_Platform_t* robot, int32_t distance) {

    if( xSemaphoreTake(m_rob_state.activate_mode, 10) == pdPASS){

        m_rob_state.distance.mode_on = true;
        m_rob_state.distance.actual_distance = 0;
        m_rob_state.distance.setpoint_distance = distance;
        m_rob_state.distance.start_time = HAL_GetTick()/portTICK_PERIOD_MS;
        m_rob_state.distance.current_time = m_rob_state.distance.start_time;

        xSemaphoreGive(m_rob_state.activate_mode);

        set_rob_state(ROB_IN_PROGRESS);
    }
}

void Robot_StartTimer(Mobile_Platform_t* robot, uint32_t ms)
{
    if( xSemaphoreTake(m_rob_state.activate_mode, 10) == pdPASS){

        m_rob_state.timer.mode_on = true;
        m_rob_state.timer.start_time = (uint32_t)(HAL_GetTick()/portTICK_PERIOD_MS);
        m_rob_state.timer.stop_time = m_rob_state.timer.start_time + ms;

        xSemaphoreGive(m_rob_state.activate_mode);

        set_rob_state(ROB_IN_PROGRESS);
    }
}

void Robot_Task(Mobile_Platform_t* robot) {

    if (m_rob_state.distance.mode_on){
        if (m_rob_state.distance.actual_distance >= m_rob_state.distance.setpoint_distance){
            Robot_Stop(robot);
        }
    }

    if (m_rob_state.timer.mode_on){
        uint32_t dt = HAL_GetTick();
        if (dt - m_rob_state.timer.start_time >= m_rob_state.timer.stop_time){
            Robot_Stop(robot);
        }
    }

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = robot->motors[idx];

        motor_task(mot_handle);
    }
}

void Robot_Stop(Mobile_Platform_t* robot) {

    if( xSemaphoreTake(m_rob_state.activate_mode, 10) == pdPASS){

        m_rob_state.distance.mode_on = false;
        m_rob_state.distance.actual_distance = 0;
        m_rob_state.distance.setpoint_distance = 0;
        m_rob_state.distance.start_time = 0;
        m_rob_state.distance.current_time = 0;


        m_rob_state.timer.mode_on = false;
        m_rob_state.timer.start_time = 0;
        m_rob_state.timer.stop_time = 0;

        robot->speed_setpoint = 0;

        for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
            motor_handle_t* mot_handle = robot->motors[idx];

            motor_stop(mot_handle);
        }

        xSemaphoreGive(m_rob_state.activate_mode);

        set_rob_state(ROB_IDLE);
    }
}

robot_status_t Robot_Status(Mobile_Platform_t* robot) {
    return m_rob_state.status;
}
