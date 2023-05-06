#ifndef ROBOT_APP_H_
#define ROBOT_APP_H_

#include <stdint.h>
#include "robot.h"
#include "commands.h"

extern Mobile_Platform_t robot;

/**
 * @brief file contains Robot macros (functions wrappers) for application creation
 * 
 */

#define ROBOT_MOVE_DISTANCE(distance, speed) \
    do{ \
        command_t cmd; \
        cmd.type = RUN_FOR_DIST; \
        cmd.distance = distance; \
        cmd.speed = speed; \
        command_add(cmd); \
    }while(0)


#define ROBOT_MOVE_SPEED(speed, time) \
    do{ \
        command_t cmd; \
        cmd.type = RUN_FOR_TIME; \
        cmd.speed = speed; \
        cmd.time = time; \
        command_add(cmd); \
    }while(0)


#define ROBOT_MOVE_TO_POINT(speed, X, Y) \
    do{ \
        command_t cmd; \
        cmd.type = RUN_TO_POINT; \
        cmd.speed = speed; \
        cmd.point.x_pos = X; \
        cmd.point.y_pos = Y; \
        command_add(cmd); \
    }while(0)


#define ROBOT_WAIT(time) \
    do{ \
        command_t cmd; \
        cmd.type = WAIT_TIME; \
        cmd.time = time; \
        command_add(cmd); \
    }while(0)


#define ROBOT_ON() \
    do{ \
        command_t cmd; \
        cmd.type = START_ROB; \
        command_add(cmd); \
    }while(0)


#define ROBOT_OFF() \
    do{ \
        command_t cmd; \
        cmd.type = STOP_ROB; \
        command_add(cmd); \
    }while(0)

#endif
