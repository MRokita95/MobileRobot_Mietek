#ifndef ROBOT_H_
#define ROBOT_H_

#include "motor_handle.h"

#define MOTORS_CNT 4


typedef struct{

    motor_handle_t* motors[MOTORS_CNT];
    
} Mobile_Platform_t;


void Robot_Init(Mobile_Platform_t* robot);

void Robot_GetSpeed(Mobile_Platform_t* robot);

void Robot_SetPath(Mobile_Platform_t* robot, int32_t speed, float angle);

void Robot_Task(Mobile_Platform_t* robot);

#endif