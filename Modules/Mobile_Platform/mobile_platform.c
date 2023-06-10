#include "robot.h"
#include "mobile_platform.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdint.h>

#define BIND_CONST -2

#define RAD_TO_DEG(rad) rad * 180.f / PI 

#define DEG_TO_RAD(deg) deg * PI / 180.f 

#define RPM_SPEED(linear_speed) (int32_t)(linear_speed * SECOND_IN_MINUTE / (int32_t)(PI * WHEEL_DIAMATER))

#define MM_S_SPEED(rpm_speed) (float)(rpm_speed) * PI * WHEEL_DIAMATER / SECOND_IN_MINUTE

#define ROUND_FLOAT(value) (value < 0) ? (value - 0.5) : (value + 0.5)



enum{
    right = 0,
    left,
};


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
            .integratorLimit = 60,
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
            .integratorLimit = 60,
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

 struct robot_internal_state_t{
    robot_status_t status;
    robot_mode_t mode;
    robot_mode_t next_mode;
    uint32_t current_time;
    distance_mode_t distance;
    time_mode_t timer;
    rob_coord_t coordinares;
    euler_angles_t imu_orient;
    rob_coord_t setpoint_coord;
    rob_angle_t orient;
    float single_wheel_speed[4];    /*0 - front right, 1 - front left, 2,3 - not used*/
    float actual_linear_speed;
    SemaphoreHandle_t activate_mode;
    SemaphoreHandle_t evaluate_status;
};

static reference_motor_t ref_table[MOTORS_CNT];



//private declarations
static void set_rob_mode(Mobile_Platform_t* robot, robot_mode_t mode);
//TODO: add here another declarations

static inline int32_t max_signed(int32_t v1, int32_t v2) {

    if((v1 > v2) && (v1 < 0)) { 
        return v2; 
    } else if ((v2 > v1) && (v2 < 0)) { 
        return v1;
    } else {
        return (v1 > v2) ? v1 : v2;
    }
}

static void set_angle(Mobile_Platform_t* robot, int32_t speed, float angle){
    
    set_rob_mode(robot, ORIENT_MODE);
    robot->handle->orient.mode_on = true;
    robot->handle->orient.setpoint = angle;

    if (robot->handle->orient.setpoint < 0.f) {
        robot->handle->orient.setpoint += 360.f;
    }

    int32_t speed_setpoint = 0;
    if (robot->handle->orient.setpoint > robot->handle->orient.actual){
        robot->handle->orient.clockwise_on = true;
        speed_setpoint = speed;
    } else {
        speed_setpoint = -speed;
    }

    Robot_SetWheelSpeed(robot, speed_setpoint, -speed_setpoint);
}

static void eval_rob_state(Mobile_Platform_t* robot) {


    if (Robot_ActiveMode(robot) == ORIENT_MODE){
        if ((robot->handle->orient.clockwise_on && (robot->handle->orient.actual >= robot->handle->orient.setpoint)) ||
            (!robot->handle->orient.clockwise_on && (robot->handle->orient.actual <= robot->handle->orient.setpoint)) ){
            
            //save current robot speed setpoint
            int32_t speed_setpoint = robot->speed_setpoint;

            Robot_Stop(robot);

            //Preset before movement to the point
            if (robot->handle->distance.preset){
                int32_t distance = (int32_t)sqrt((robot->handle->setpoint_coord.x_pos*robot->handle->setpoint_coord.x_pos) +
                                                    (robot->handle->setpoint_coord.y_pos*robot->handle->setpoint_coord.y_pos));

                //TODO: fixme :(
                if ((robot->handle->setpoint_coord.x_pos < robot->handle->coordinares.x_pos) ||
                    (robot->handle->setpoint_coord.y_pos < robot->handle->coordinares.y_pos)) {
                    distance = -distance;
                }

                Robot_SetSpeed(robot, speed_setpoint);
                Robot_SetDistance(robot, distance);

                robot->handle->distance.preset = false;
            } 
            //Otherway - End Command with proper status
            else {
                Robot_Stop(robot);
                End_Command_Execution(DONE_OK);

                //TODO:
                //End_Command_Execution(TIMEOUT);
                //End_Command_Execution(ERR);
            }
        }
    }


   if (Robot_ActiveMode(robot) == DISTANCE_MODE){
        if (abs(robot->handle->distance.actual_distance) >= abs(robot->handle->distance.setpoint_distance)){
            
            robot->handle->setpoint_coord.x_pos = 0;
            robot->handle->setpoint_coord.y_pos = 0;

            Robot_Stop(robot);
            End_Command_Execution(DONE_OK);

            //TODO:
            //End_Command_Execution(TIMEOUT);
            //End_Command_Execution(ERR);
        } 
        else if (abs(robot->handle->distance.actual_distance) >= abs(robot->handle->distance.ramp_distance) && 
            (!robot->handle->distance.slow_ramp_on) && (robot->handle->distance.ramp_distance > 1.f)) {
            
            Robot_SetSpeed(robot, robot->speed_setpoint/2);
            robot->handle->distance.slow_ramp_on = true;
        }
    }

    if (Robot_ActiveMode(robot) == TIMER_MODE){
        uint32_t dt = (uint32_t)(HAL_GetTick()/portTICK_PERIOD_MS);
        if (dt - robot->handle->timer.start_time >= robot->handle->timer.stop_time){
            Robot_Stop(robot);
            End_Command_Execution(DONE_OK);

            //TODO:
            //End_Command_Execution(ERR);
        }
    }

    if (Robot_ActiveMode(robot) == MANUAL_MODE){
        //for now nothing
    }

    if (Robot_ActiveMode(robot) == AUTONOMOUS_MODE){
        //end mode here
    }
}


static void eval_motion_status(Mobile_Platform_t* robot) {

    float actual_dist = 0;
    float actual_linear_speed;
    float sum_speed = 0;
    int32_t encoders = 0;

    uint32_t dt = (uint32_t)(HAL_GetTick()/portTICK_PERIOD_MS) - (robot->handle->current_time);
    float dt_s = (float)dt/1000.f;

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = robot->motors[idx];

        if (mot_handle->encoder_present){
            actual_linear_speed = MM_S_SPEED(mot_handle->encoder.act_speed);

            robot->handle->single_wheel_speed[idx] = actual_linear_speed;
            sum_speed += actual_linear_speed;
            actual_dist += (actual_linear_speed * dt_s);
            encoders++;
        } 
    }
    
    /* general mobile robot speed*/
    robot->actual_speed = (int32_t)(sum_speed / encoders);

    /* detailed speed used for coordinates calculation */
    /* TODO: ADD X Y Z CALCULATION BASED ON ACTUAL ANGLE !!!!!!! - coordinates must stay the same */
    // if (!robot->handle->distance.preset){
    //     robot->handle->coordinares.x_pos += (int32_t)(dt_s * (int32_t)(robot->handle->single_wheel_speed[right]/2.f + robot->handle->single_wheel_speed[left]/2.f));
    //     robot->handle->coordinares.y_pos += (int32_t)(dt_s * (int32_t)(robot->handle->single_wheel_speed[right] - robot->handle->single_wheel_speed[left]));
    //     robot->handle->coordinares.z_pos = 0; /*disabled until IMU integration*/
    // }

    if(robot->handle->orient.mode_on){
        float d_orient = RAD_TO_DEG((float)atan((abs(dt_s*robot->handle->single_wheel_speed[right])+
                                            abs(dt_s*robot->handle->single_wheel_speed[left]))/
                                            MOBILE_PLATFORM_WIDTH/2));
        if(robot->handle->orient.clockwise_on){
            robot->handle->orient.actual += d_orient;
        } else {
            robot->handle->orient.actual -= d_orient;
        }
    }

    /*zero the orientation*/
    if (robot->handle->orient.actual >= 360.0){
        robot->handle->orient.actual -= 360.0;
    }

    //cheat a little :D
    robot->handle->imu_orient.yaw = robot->handle->orient.actual;

    // robot->handle->coordinares.x_pos -= (int32_t)((dt_s * (int32_t)((robot->handle->single_wheel_speed[right] +
    //                                     robot->handle->single_wheel_speed[left])/2.f)) *
    //                                     cos((double)DEG_TO_RAD(robot->handle->orient.actual)));
    // robot->handle->coordinares.y_pos -= (int32_t)((dt_s * (int32_t)((robot->handle->single_wheel_speed[right] +
    //                                     robot->handle->single_wheel_speed[left])/2.f))*
    //                                     sin((double)DEG_TO_RAD(robot->handle->orient.actual)));

    int32_t dx = (int32_t)ROUND_FLOAT((actual_dist / encoders) * cos((double)DEG_TO_RAD((int16_t)robot->handle->orient.actual)));
    int32_t dy = (int32_t)ROUND_FLOAT((actual_dist / encoders) * sin((double)DEG_TO_RAD((int16_t)robot->handle->orient.actual)));
    if (robot->speed_setpoint > 0)
    {
        robot->handle->coordinares.x_pos += dx;
        robot->handle->coordinares.y_pos += dy;
        robot->handle->coordinares.z_pos = 0; /*disabled until IMU integration*/
    } 
    else if (robot->speed_setpoint < 0) {
        robot->handle->coordinares.x_pos += dx;
        robot->handle->coordinares.y_pos += dy;
        robot->handle->coordinares.z_pos = 0; /*disabled until IMU integration*/
    }

    
    robot->handle->current_time += dt;


    if (!robot->handle->distance.mode_on){
        return;
    }
    robot->handle->distance.actual_distance = robot->handle->distance.actual_distance + (actual_dist / encoders);
    robot->handle->distance.current_time += dt;

    robot->current_distance =  robot->handle->distance.actual_distance;
}


static void set_rob_state(Mobile_Platform_t* robot, robot_status_t status){
    if( xSemaphoreTake(robot->handle->evaluate_status, 10) == pdPASS){
        robot->handle->status = status;
        xSemaphoreGive(robot->handle->evaluate_status);
    }
}



static void set_rob_mode(Mobile_Platform_t* robot, robot_mode_t mode){
    robot->handle->mode = mode;
}


// static uint8_t handle_manual_ctrl(manual_ctrl_command_t command){
//     uint8_t up = (command.up) ? 1u : 0u;
//     uint8_t right = (command.right) ? 1u : 0u;
//     uint8_t left = (command.left) ? 1u : 0u;
//     uint8_t down = (command.down) ? 1u : 0u;

//     uint8_t control = 0;
//     control |= up;
//     control |= right << 1;
//     control |= left << 1;
//     control |= down << 1;
//     return control;
// }

/**
* PUBLIC FUNCTIONS
 */


void Robot_Init(Mobile_Platform_t* robot){

    /*create robot internal state handle*/
    robot->handle = malloc(sizeof(struct robot_internal_state_t));
    if (robot->handle == NULL){
        return;
    }

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

    robot->handle->activate_mode = xSemaphoreCreateMutex();
    robot->handle->evaluate_status = xSemaphoreCreateMutex();

    robot->handle->orient.actual = 0.0;
    robot->handle->distance.current_time = (uint32_t)(HAL_GetTick()/portTICK_PERIOD_MS);
    robot->handle->current_time = (uint32_t)(HAL_GetTick()/portTICK_PERIOD_MS);

    HK_Init(robot);

    Robot_Stop(robot);
}

void Robot_UpdateMotionStatus(Mobile_Platform_t* robot){
    
    //IMU euler orient
    Sensor_GetValue(IMU, &robot->handle->imu_orient);

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = robot->motors[idx];

        if (mot_handle->encoder_present){
            motor_update_speed(mot_handle);
        } 
    }
    eval_motion_status(robot);
}


void Robot_SetSpeed(Mobile_Platform_t* robot, int32_t speed) {

    robot->speed_setpoint = speed;
    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = robot->motors[idx];

        int32_t rpm_setpoint = RPM_SPEED(robot->speed_setpoint);
        motor_set_speed(mot_handle, rpm_setpoint);
    }
}

void Robot_SetWheelSpeed(Mobile_Platform_t* robot, int32_t speed_right, int32_t speed_left) {


    motor_handle_t* mot_handle = robot->motors[right];
    int32_t rpm_setpoint = RPM_SPEED(speed_right);
    motor_set_speed(mot_handle, rpm_setpoint);

    mot_handle = robot->motors[left];
    rpm_setpoint = RPM_SPEED(speed_left);
    motor_set_speed(mot_handle, rpm_setpoint);
}

void Robot_SetDistance(Mobile_Platform_t* robot, float distance) {

    if( xSemaphoreTake(robot->handle->activate_mode, 10) == pdPASS){
        
        set_rob_mode(robot, DISTANCE_MODE);
        robot->handle->distance.mode_on = true;
        robot->handle->distance.actual_distance = 0;
        robot->handle->distance.setpoint_distance = distance;
        robot->handle->distance.start_time = HAL_GetTick()/portTICK_PERIOD_MS;
        robot->handle->distance.current_time = robot->handle->distance.start_time;

        if (RAMP_DIST != 0 && (float)RAMP_DIST < distance){
            robot->handle->distance.ramp_distance = distance - (float)RAMP_DIST;
        }

        if (distance < 0.f){
            Robot_SetSpeed(robot, -robot->speed_setpoint);
        }
        xSemaphoreGive(robot->handle->activate_mode);

        set_rob_state(robot, ROB_IN_PROGRESS);

        HK_Setpoints(&robot->handle->distance.setpoint_distance, DISTANCE);
    }
}

void Robot_StartTimer(Mobile_Platform_t* robot, uint32_t ms)
{
    if( xSemaphoreTake(robot->handle->activate_mode, 10) == pdPASS){

        set_rob_mode(robot, TIMER_MODE);
        robot->handle->timer.mode_on = true;
        robot->handle->timer.start_time = (uint32_t)(HAL_GetTick()/portTICK_PERIOD_MS);
        robot->handle->timer.stop_time = ms;

        xSemaphoreGive(robot->handle->activate_mode);

        set_rob_state(robot, ROB_IN_PROGRESS);

        uint32_t setpoint = ms;
        HK_Setpoints(&setpoint, TIMER);
    }
}

void Robot_MoveToPoint(Mobile_Platform_t* robot, int32_t speed, int32_t x_pos, int32_t y_pos) {

    if( xSemaphoreTake(robot->handle->activate_mode, 10) == pdPASS){

        set_rob_mode(robot, POINT_MODE);
        float angle = RAD_TO_DEG((float)atan((double)(y_pos)/(double)(x_pos)));

        //Get relative distance to the position
        robot->handle->setpoint_coord.x_pos = x_pos - robot->handle->coordinares.x_pos;
        robot->handle->setpoint_coord.y_pos = y_pos - robot->handle->coordinares.y_pos;
        robot->speed_setpoint = speed;

        robot->handle->distance.preset = true;

        xSemaphoreGive(robot->handle->activate_mode);

        set_rob_state(robot, ROB_IN_PROGRESS);
        set_angle(robot, ALIGN_SPEED, angle);

        HK_Setpoints(&robot->handle->setpoint_coord, POINT);
    }
}

void Robot_Rotate(Mobile_Platform_t* robot, int32_t speed, int16_t angle_setpoint) {

    if( xSemaphoreTake(robot->handle->activate_mode, 10) == pdPASS){

        robot->speed_setpoint = speed;

        xSemaphoreGive(robot->handle->activate_mode);

        set_rob_state(robot, ROB_IN_PROGRESS);
        set_angle(robot, speed, (float)angle_setpoint);

        HK_Setpoints(&robot->handle->orient.setpoint, ROTATE);
    }
}

void Robot_Stop(Mobile_Platform_t* robot) {

    if( xSemaphoreTake(robot->handle->activate_mode, 10) == pdPASS){
        set_rob_mode(robot, NONE);

        robot->handle->distance.mode_on = false;
        robot->handle->distance.actual_distance = 0;
        robot->handle->distance.setpoint_distance = 0;
        robot->handle->distance.start_time = 0;
        robot->handle->distance.current_time = 0;
        robot->handle->distance.slow_ramp_on = false;
        robot->handle->distance.ramp_distance = 0.f;


        robot->handle->timer.mode_on = false;
        robot->handle->timer.start_time = 0;
        robot->handle->timer.stop_time = 0;

        robot->handle->orient.mode_on = false;
        robot->handle->orient.clockwise_on = false;
        robot->handle->orient.setpoint = 0.0;


        robot->speed_setpoint = 0;

        for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
            motor_handle_t* mot_handle = robot->motors[idx];

            motor_stop(mot_handle);
        }

        xSemaphoreGive(robot->handle->activate_mode);

        set_rob_state(robot, ROB_IDLE);
    }
}


void Robot_Task(Mobile_Platform_t* robot) {

    if (Robot_Status(robot) == ROB_IDLE){

        bool cmd_on = Execute_Command();

        if (cmd_on){ 
            ROB_DEBUG("RUNNING COMMAND...\r\n");
        } 
        
    } else {
        eval_rob_state(robot);
    }

    for (uint8_t idx = 0; idx < MOTORS_CNT; idx++){
        motor_handle_t* mot_handle = robot->motors[idx];

        motor_task(mot_handle);
    }

    //HouseKeeping Queue Send
    HK_Update(robot);

}

void Robot_ManualCtrl(Mobile_Platform_t* robot, manual_ctrl_command_t ctrl)
{
    enum {
        UP = 1,
        RIGHT = UP << 1,
        LEFT = RIGHT << 1,
        DOWN = LEFT << 1,
    };
    uint8_t manual_control; //= handle_manual_ctrl(ctrl);
    
    if (ctrl.bit.up && !(ctrl.bit.left || ctrl.bit.right || ctrl.bit.down)){
        Robot_SetSpeed(robot, MANUAL_SPEED);
    }
    else if((ctrl.bit.up && ctrl.bit.right) && !(ctrl.bit.left || ctrl.bit.down)){
        Robot_SetWheelSpeed(robot, MANUAL_SPEED/2, MANUAL_SPEED);
    }
    else if((ctrl.bit.up && ctrl.bit.left) && !(ctrl.bit.right || ctrl.bit.down)){
        Robot_SetWheelSpeed(robot, MANUAL_SPEED, MANUAL_SPEED/2);
    }
    else if((ctrl.bit.right && ctrl.bit.left) && !(ctrl.bit.up || ctrl.bit.down)){
        Robot_SetWheelSpeed(robot, MANUAL_SPEED, -MANUAL_SPEED);
    }
    else if((ctrl.bit.up && ctrl.bit.down) && !(ctrl.bit.left || ctrl.bit.right)){
        Robot_SetWheelSpeed(robot, -MANUAL_SPEED, MANUAL_SPEED);
    }
    else if (ctrl.bit.down && !(ctrl.bit.left || ctrl.bit.right || ctrl.bit.up)){
        Robot_SetSpeed(robot, -MANUAL_SPEED);
    }
    else if((ctrl.bit.down && ctrl.bit.right) && !(ctrl.bit.left || ctrl.bit.up)){
        Robot_SetWheelSpeed(robot, -MANUAL_SPEED/2, -MANUAL_SPEED);
    }
    else if((ctrl.bit.down && ctrl.bit.left) && !(ctrl.bit.right || ctrl.bit.up)){
        Robot_SetWheelSpeed(robot, -MANUAL_SPEED, -MANUAL_SPEED/2);
    }
    else {
        Robot_SetSpeed(robot, 0);
    }
}

void Robot_SetMode(Mobile_Platform_t* robot, robot_mode_t mode) {
    bool mode_is_none = Robot_ActiveMode(robot) == NONE;
    if ((mode != MANUAL_MODE) || (mode != AUTONOMOUS_MODE) || !mode_is_none){
        return;
    }
    set_rob_mode(robot, mode);
}

robot_status_t Robot_Status(Mobile_Platform_t* robot) {
    return robot->handle->status;
}

robot_status_t Robot_ActiveMode(Mobile_Platform_t* robot) {
    return robot->handle->mode;
}

rob_coord_t Robot_GetCoord(Mobile_Platform_t* robot) {
    return robot->handle->coordinares;
}

euler_angles_t Robot_GetOrient(Mobile_Platform_t* robot) {
    return robot->handle->imu_orient;
}
