#include "robot.h"
#include "mobile_platform.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

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
            .integratorLimit = 100,
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
            .integratorLimit = 100,
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
    movement_phase_t mov_phase;
    float single_wheel_speed[4];    /*0 - front right, 1 - front left, 2,3 - not used*/
    float actual_linear_speed;
    float acc_setpoint;
    SemaphoreHandle_t activate_mode;
    SemaphoreHandle_t evaluate_status;
};

static reference_motor_t ref_table[MOTORS_CNT];



//private declarations
static void set_rob_mode(Mobile_Platform_t* robot, robot_mode_t mode);
static int32_t speed_profiler(Mobile_Platform_t* robot);
static inline void clamp_orient(float* orient);
static void set_rob_state(Mobile_Platform_t* robot, robot_status_t status);

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
    
    float tol = 1.f;
    float ang_diff = robot->handle->orient.actual - angle;

    if (fabsf(ang_diff) <= tol) {
        set_rob_state(robot, ROB_IDLE);
        return;
    }

    set_rob_mode(robot, ORIENT_MODE);
    robot->handle->orient.mode_on = true;
    robot->handle->orient.setpoint = angle;

    // if (robot->handle->setpoint_coord.x_pos < 0 && robot->handle->setpoint_coord.y_pos < 0){
    //     robot->handle->orient.setpoint -= 90.f;

    // } else if (robot->handle->setpoint_coord.x_pos < 0 && robot->handle->setpoint_coord.y_pos > 0){
    //     robot->handle->orient.setpoint += 90.f;
    // }

    clamp_orient(&robot->handle->orient.setpoint);

    robot->handle->orient.clockwise_on = (robot->handle->orient.actual > robot->handle->orient.setpoint) ? true : false;


    int32_t speed_setpoint = 0;
    if (robot->handle->orient.clockwise_on){
        speed_setpoint = speed;
    } else {
        speed_setpoint = -speed;
    }

    Robot_SetWheelSpeed(robot, -speed_setpoint, speed_setpoint);

    robot->handle->mov_phase = MOVEMENT;
}

static inline bool orient_reached(float actual, float setpoint, bool clockwise_on){

    bool stop_cond = false;
    float tol = 1.f;

    if (clockwise_on){
        stop_cond = ((actual - tol) <= setpoint) ? true : false;
    } else {
        stop_cond = ((actual + tol) >= setpoint) ? true : false;
    }

    return stop_cond;
}

static void eval_rob_state(Mobile_Platform_t* robot) {


    if (Robot_ActiveMode(robot) == ORIENT_MODE){
        if (orient_reached(robot->handle->orient.actual, robot->handle->orient.setpoint, robot->handle->orient.clockwise_on)){
            
            //save current robot speed setpoint
            int32_t speed_setpoint = robot->speed_setpoint;

            Robot_Stop(robot);

            //Preset before movement to the point
            if (robot->handle->distance.preset){
                float distance = (float)sqrt((robot->handle->setpoint_coord.x_pos*robot->handle->setpoint_coord.x_pos) +
                                                    (robot->handle->setpoint_coord.y_pos*robot->handle->setpoint_coord.y_pos));


                Robot_SetSpeed(robot, speed_setpoint);
                Robot_SetDistance(robot, distance);

                robot->handle->distance.preset = false;
            } 
            //Otherway - End Command with proper status
            else {
                Robot_Stop(robot);
                End_Command_Execution(robot, DONE_OK);

                //TODO:
                //End_Command_Execution(TIMEOUT);
                //End_Command_Execution(ERR);
            }
        }
    }


   if (Robot_ActiveMode(robot) == DISTANCE_MODE){

        int32_t next_speed_setpoint = speed_profiler(robot);

        if (robot->handle->mov_phase == BRAKE){
            Robot_SetSpeed(robot, next_speed_setpoint);
        } 
        else if (robot->handle->mov_phase == STOP){
            Robot_Stop(robot);
            End_Command_Execution(robot, DONE_OK);
        }
    }

    if (Robot_ActiveMode(robot) == TIMER_MODE){
        uint32_t dt = (uint32_t)(HAL_GetTick()/portTICK_PERIOD_MS);
        if (dt - robot->handle->timer.start_time >= robot->handle->timer.stop_time){
            Robot_Stop(robot);
            End_Command_Execution(robot, DONE_OK);

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

static inline void clamp_orient(float* orient){

    if (*orient >= 180.0){
        float diff = 180.f - *orient;
        *orient = -180.0 + diff;

    } else if (*orient <= -180.0){
        float diff = -180.f + *orient;
        *orient =  180.0 - diff;
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


    if(robot->handle->orient.mode_on){
        float d_orient = RAD_TO_DEG((float)atan((abs(dt_s*robot->handle->single_wheel_speed[right])+
                                            abs(dt_s*robot->handle->single_wheel_speed[left]))/
                                            MOBILE_PLATFORM_WIDTH/2));
        if(robot->handle->orient.clockwise_on){
            robot->handle->orient.actual -= d_orient;
        } else {
            robot->handle->orient.actual += d_orient;
        }
    }

    /*zero the orientation*/
    clamp_orient(&robot->handle->orient.actual);

    //Magnetometer works but not calibrated correctly
    //robot->handle->imu_orient.yaw = robot->handle->orient.actual;


    int32_t dx = (int32_t)ROUND_FLOAT((actual_dist / (float)encoders) * cos((double)DEG_TO_RAD((int16_t)robot->handle->orient.actual)));
    int32_t dy = (int32_t)ROUND_FLOAT((actual_dist / (float)encoders) * sin((double)DEG_TO_RAD((int16_t)robot->handle->orient.actual)));
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

    robot->current_distance =  robot->handle->distance.actual_distance;
}

static inline int32_t next_speed_setpoint(){
    return 0;   //mock for now
}

/**
 * @brief return next speed setpoint based on set accel and current distance from rob->handle
 * 
 * @param robot 
 * @return int32_t 
 */
static int32_t speed_profiler(Mobile_Platform_t* robot){

    const float tol = 1.0f;

    int32_t speed_setp = robot->speed_setpoint;
    int32_t next_speed = next_speed_setpoint();

    float dist_left = robot->handle->distance.setpoint_distance - robot->handle->distance.actual_distance;
    float braking_dist = (float)((robot->actual_speed - next_speed) * (robot->actual_speed + next_speed)) / (2.f * robot->handle->acc_setpoint);


    if (robot->handle->mov_phase == BRAKE){

        uint32_t dt = (uint32_t)(HAL_GetTick()/portTICK_PERIOD_MS) - robot->handle->distance.current_time;

        float dt_s = (float)dt/1000.f;
        if (abs(speed_setp) > abs(next_speed)){

            speed_setp = speed_setp - (int32_t)((float)dt_s * robot->handle->acc_setpoint);

            if (abs(speed_setp) <= abs(next_speed)){
                speed_setp = next_speed;
            }
        }
    }

    if (robot->handle->mov_phase != BRAKE){

        if (dist_left <= braking_dist){
            robot->handle->mov_phase = BRAKE;
        }
    }

    //additional check for safety - independent from mov_phase
    if ((abs(robot->handle->distance.actual_distance) >= abs(robot->handle->distance.setpoint_distance + tol)) ||
        ((robot->handle->mov_phase == BRAKE) && speed_setp == 0)
    ){
            
        robot->handle->setpoint_coord.x_pos = 0;
        robot->handle->setpoint_coord.y_pos = 0;
        robot->handle->setpoint_coord.z_pos = 0;
        robot->handle->mov_phase = STOP;
        speed_setp = 0;
    }

    robot->handle->distance.current_time = (uint32_t)(HAL_GetTick()/portTICK_PERIOD_MS);

    return speed_setp;
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

    robot->handle->coordinares.x_pos = 0;
    robot->handle->coordinares.y_pos = 0;
    robot->handle->coordinares.z_pos = 0;

    robot->handle->actual_linear_speed = 0.f;

    robot->handle->mov_phase = STOP;

    robot->handle->acc_setpoint = 100.f;  //TODO: default value

    robot->handle->activate_mode = xSemaphoreCreateMutex();
    robot->handle->evaluate_status = xSemaphoreCreateMutex();

    robot->handle->orient.actual = 0.f;
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

        robot->handle->mov_phase = MOVEMENT;

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

static rob_coord_t calculate_coord_offset(rob_coord_t actual, int32_t x_pos, int32_t y_pos){

    rob_coord_t delta;
    delta.z_pos = 0;

    if (x_pos < 0){
        delta.x_pos = (actual.x_pos <= 0) ? x_pos - actual.x_pos : actual.x_pos - x_pos;
    } else {
        delta.x_pos = x_pos - actual.x_pos;
    }

    if (y_pos < 0){
        delta.y_pos = (actual.y_pos <= 0) ? y_pos - actual.y_pos : actual.y_pos - y_pos;
    } else {
        delta.y_pos = y_pos - actual.y_pos;
    }

    return delta;
}

/**
 * @brief Move to point in global coordinates
 * 
 * @param robot 
 * @param speed 
 * @param x_pos 
 * @param y_pos 
 */
void Robot_MoveToPoint(Mobile_Platform_t* robot, int32_t speed, int32_t x_pos, int32_t y_pos) {

    if( xSemaphoreTake(robot->handle->activate_mode, 10) == pdPASS){

        set_rob_mode(robot, POINT_MODE);

        //Get relative distance to the position
        rob_coord_t offset = calculate_coord_offset(robot->handle->coordinares, x_pos, y_pos);

        robot->handle->setpoint_coord.x_pos = offset.x_pos;
        robot->handle->setpoint_coord.y_pos = offset.y_pos;
        robot->speed_setpoint = speed;

        float angle = RAD_TO_DEG((float)atan2f((float)(offset.y_pos), (float)(offset.x_pos)));
        robot->handle->distance.preset = true;

        xSemaphoreGive(robot->handle->activate_mode);

        set_rob_state(robot, ROB_IN_PROGRESS);

        set_angle(robot, ALIGN_SPEED, angle);

        //go to point is handled internally
        if (robot->handle->mode == ORIENT_MODE){
            return;
        }

        float distance = (float)sqrt((offset.x_pos*offset.x_pos) + (offset.y_pos*offset.y_pos));

        Robot_SetSpeed(robot, speed);
        Robot_SetDistance(robot, distance);


        HK_Setpoints(&robot->handle->setpoint_coord, POINT);
    }
}

void Robot_Rotate(Mobile_Platform_t* robot, int32_t speed, int16_t angle_setpoint) {

    if( xSemaphoreTake(robot->handle->activate_mode, 10) == pdPASS){

        robot->speed_setpoint = speed;

        xSemaphoreGive(robot->handle->activate_mode);

        set_angle(robot, speed, (float)angle_setpoint);

        if (robot->handle->mov_phase == MOVEMENT){
            set_rob_state(robot, ROB_IN_PROGRESS);
            HK_Setpoints(&robot->handle->orient.setpoint, ROTATE);
        }

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

        robot->handle->mov_phase = STOP;

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

        bool cmd_on = Execute_Command(robot);

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
