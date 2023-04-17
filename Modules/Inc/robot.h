#ifndef ROBOT_H_
#define ROBOT_H_

#include "motor.h"
#include <stdint.h>
#include <stdbool.h>

#define MOTORS_CNT 4


typedef struct{

    int32_t speed_setpoint;     // [mm/s]
    int32_t actual_speed;       // [mm/s]
    uint32_t current_distance;  // [mm] from last movement command
    motor_handle_t* motors[MOTORS_CNT];
    
} Mobile_Platform_t;


typedef enum robot_status{
    ROB_OFF = 0,
    ROB_IDLE,
    ROB_IN_PROGRESS,
} robot_status_t;


void Robot_Init(Mobile_Platform_t* robot);

void Robot_UpdateMotionStatus(Mobile_Platform_t* robot);

void Robot_SetPath(Mobile_Platform_t* robot, int32_t speed, float angle);

void Robot_SetDistance(Mobile_Platform_t* robot, int32_t distance);

void Robot_StartTimer(Mobile_Platform_t* robot, uint32_t ms);

void Robot_Task(Mobile_Platform_t* robot);

void Robot_Stop(Mobile_Platform_t* robot);

robot_status_t Robot_Status(Mobile_Platform_t* robot);

#endif