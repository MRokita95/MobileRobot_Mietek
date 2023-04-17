#ifndef ROBOT_APP_H_
#define ROBOT_APP_H_

#include <stdint.h>
#include "robot.h"

extern Mobile_Platform_t robot;

static uint32_t program_counter = 1;
static uint32_t current_step = 1;

/**
 * @brief file contains Robot macros (functions wrappers) for application creation
 * 
 */

#define ROBOT_MOVE_DISTANCE(distance, speed) \
    do{ \
        static uint32_t step_no = 0u; \
        if (1 == 1) { \
            if (Robot_Status(&robot) == ROB_IDLE) { \
                Robot_SetPath(&robot, speed, 0); \
                Robot_SetDistance(&robot, distance); \
                current_step++; \
            } \
        } \
        else { \
            step_no = program_counter; \
            program_counter++; \
        } \
    }while(0)


#define ROBOT_MOVE_SPEED(speed, angle) \
    do{ \
        static uint32_t step_no = 0u; \
        if (1 == 1) { \
            if (Robot_Status(&robot) == ROB_IDLE) { \
                Robot_SetPath(&robot, speed, angle); \
                current_step++; \
            } \
        } \
        else { \
            step_no = program_counter; \
            program_counter++; \
        } \
    }while(0)


#define ROBOT_MOVE_SPEED_TIME(speed, angle, timeout) \
    do{ \
        static uint32_t step_no = 0u; \
        if (1 == 1) { \
            if (Robot_Status(&robot) == ROB_IDLE) { \
                Robot_SetPath(&robot, speed, angle); \
                Robot_StartTimer(&robot, timeout); \
                current_step++; \
            } \
        } \
        else { \
            step_no = program_counter; \
            program_counter++; \
        } \
    }while(0)


#endif
