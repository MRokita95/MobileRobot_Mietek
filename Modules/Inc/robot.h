#ifndef ROBOT_H_
#define ROBOT_H_

#include "motor.h"
#include <stdint.h>
#include <stdbool.h>
#include "robot.h"
#include "sensors_common.h"

/**
 * @brief PUBLIC functions, defines and typedefs
 * 
 */

#define DEBUG_PRINT 1
#ifdef DEBUG_PRINT
    extern UART_HandleTypeDef huart2;
    extern void send_uart(UART_HandleTypeDef* uart_instance, void const * argument);
    #define ROB_DEBUG(...) send_uart(&huart2, __VA_ARGS__)
#else
    #define ROB_DEBUG(...) {}
#endif

#define MOTORS_CNT 4

#define MOBILE_PLATFORM_CNT 1

#define HK_UPDATE 1000 /*ms*/

typedef struct robot_internal_state_t* rob_handle_t;

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
    rob_handle_t handle;
    motor_handle_t* motors[MOTORS_CNT];
    
} Mobile_Platform_t;


typedef enum robot_status{
    ROB_OFF = 0,
    ROB_IDLE,
    ROB_IN_PROGRESS,
} robot_status_t;

typedef enum robot_mode{
    NONE = 0,
    TIMER_MODE,
    DISTANCE_MODE,
    POINT_MODE,
    ORIENT_MODE,
    MANUAL_MODE,
    AUTONOMOUS_MODE,
} robot_mode_t;

typedef enum robot_wheel{
    RIGHT = 0,
    LEFT
}robot_wheel;


void Robot_Init(Mobile_Platform_t* robot);

void Robot_UpdateMotionStatus(Mobile_Platform_t* robot);

void Robot_Task(Mobile_Platform_t* robot);

euler_angles_t Robot_GetOrient(Mobile_Platform_t* robot);

rob_coord_t Robot_GetCoord(Mobile_Platform_t* robot);

robot_status_t Robot_Status(Mobile_Platform_t* robot);

robot_mode_t Robot_ActiveMode(Mobile_Platform_t* robot);

int32_t Robot_GetWheelSpeed(Mobile_Platform_t* robot, robot_wheel wheel);

void Robot_Stop(Mobile_Platform_t* robot);

#endif
