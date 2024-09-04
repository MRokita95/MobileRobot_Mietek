#ifndef MOBILE_PLATFORM_H_
#define MOBILE_PLATFORM_H_

#include <stdint.h>
#include <stdbool.h>
#include "commands.h"
#include "sensors_common.h"

/**
 * @brief Protected functions, definitions and typedefs
 * 
 */

#define WHEEL_DIAMATER 65.f //[mm] 
#define PI 3.14f
#define MOBILE_PLATFORM_WIDTH 168.f //MAX is 180 mm

#define ALIGN_SPEED 60
#define RAMP_DIST 0 //mm before setpoint distance

#define MANUAL_SPEED 150 // [mm/s]

#define MINIMAL_SPEED 5 // [mm/s]

typedef struct {
    bool mode_on;
    bool preset;
    bool slow_ramp_on;
    float actual_distance;   //in [mm]
    float setpoint_distance;   //in [mm]
    float ramp_distance;
    uint32_t start_time;
    uint32_t current_time;
} distance_mode_t;

typedef struct {
    bool mode_on;
    uint32_t start_time;
    uint32_t stop_time;
} time_mode_t;


typedef struct {
    bool mode_on;
    float actual;
    float setpoint;
    bool clockwise_on;
} rob_angle_t;


typedef enum{
    TIMER = 0,
    DISTANCE,
    POINT,
} current_setpoint_t;


typedef enum{
    STOP = 0,
    ACCEL,
    MOVEMENT,
    BRAKE
} movement_phase_t;


void HK_Init(Mobile_Platform_t* robot);

void HK_Update(Mobile_Platform_t* robot);

void HK_Setpoints(void* setpoint, current_setpoint_t setpoint_type);

bool Execute_Command(Mobile_Platform_t *robot);

void End_Command_Execution(Mobile_Platform_t *robot, command_status_t status);

void Robot_Rotate(Mobile_Platform_t* robot, int32_t speed, int16_t angle_setpoint);

void Robot_SetSpeed(Mobile_Platform_t* robot, int32_t speed);

void Robot_SetDistance(Mobile_Platform_t* robot, float distance);

void Robot_MoveToPoint(Mobile_Platform_t* robot, int32_t speed, int32_t x_pos, int32_t y_pos);

void Robot_StartTimer(Mobile_Platform_t* robot, uint32_t ms);

void Robot_SetMode(Mobile_Platform_t* robot, robot_mode_t mode);

void Robot_ManualCtrl(Mobile_Platform_t* robot, manual_ctrl_command_t ctrl);

#endif
