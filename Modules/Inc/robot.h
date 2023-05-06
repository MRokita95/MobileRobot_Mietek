#ifndef ROBOT_H_
#define ROBOT_H_

#include "motor.h"
#include <stdint.h>
#include <stdbool.h>

#define DEBUG_PRINT 1
#ifdef DEBUG_PRINT
    extern UART_HandleTypeDef huart2;
    extern void send_uart(UART_HandleTypeDef* uart_instance, void const * argument);
    #define ROB_DEBUG(...) send_uart(&huart2, __VA_ARGS__)
#else
    #define ROB_DEBUG(...) {}
#endif

#define MOTORS_CNT 4

#define HK_UPDATE 1000 /*ms*/

typedef struct
{
    int32_t x_pos;
    int32_t y_pos;
    int32_t z_pos;
} rob_coord_t;

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

void Robot_SetSpeed(Mobile_Platform_t* robot, int32_t speed);

void Robot_SetDistance(Mobile_Platform_t* robot, int32_t distance);

void Robot_MoveToPoint(Mobile_Platform_t* robot, int32_t speed, int32_t x_pos, int32_t y_pos);

void Robot_StartTimer(Mobile_Platform_t* robot, uint32_t ms);

void Robot_Task(Mobile_Platform_t* robot);

void Robot_Stop(Mobile_Platform_t* robot);

robot_status_t Robot_Status(Mobile_Platform_t* robot);

rob_coord_t Robot_GetCoord(Mobile_Platform_t* robot);


#endif