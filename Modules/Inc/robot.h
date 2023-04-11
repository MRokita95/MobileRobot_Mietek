#ifndef ROBOT_H_
#define ROBOT_H_

#include "motor.h"
#include <stdint.h>
#include <stdbool.h>

#define MOTORS_CNT 4

typedef struct {
    bool mode_on;
    int32_t actual_distance;   //in [mm]
    int32_t setpoint_distance;   //in [mm]
    uint32_t start_time;
    uint32_t current_time;
} distance_mode_t;

typedef struct {
    bool mode_on;
    uint32_t start_time;
    uint32_t stop_time;
} time_mode_t;


typedef struct{

    bool initialized;
    distance_mode_t distance;
    time_mode_t timer;
    motor_handle_t* motors[MOTORS_CNT];
    
} Mobile_Platform_t;


void Robot_Init(Mobile_Platform_t* robot);

void Robot_UpdateMotionStatus(Mobile_Platform_t* robot);

void Robot_SetPath(Mobile_Platform_t* robot, int32_t speed, float angle);

void Robot_SetDistance(Mobile_Platform_t* robot, int32_t distance);

void Robot_CalcDistance(Mobile_Platform_t* robot);

void Robot_StartTimer(Mobile_Platform_t* robot, uint32_t ms);

void Robot_Task(Mobile_Platform_t* robot);

void Robot_Stop(Mobile_Platform_t* robot);

#endif