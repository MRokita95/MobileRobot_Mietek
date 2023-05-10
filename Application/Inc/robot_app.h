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

#define ROBOT_MOVE_DISTANCE(setp_speed, setp_distance) \
    do{ \
        command_t cmd; \
        cmd.type = RUN_FOR_DIST; \
        cmd.distance = setp_distance; \
        cmd.speed = setp_speed; \
        command_add(cmd); \
    }while(0)


#define ROBOT_MOVE_SPEED(setp_speed, setp_time) \
    do{ \
        command_t cmd; \
        cmd.type = RUN_FOR_TIME; \
        cmd.speed = setp_speed; \
        cmd.time = setp_time; \
        command_add(cmd); \
    }while(0)


#define ROBOT_MOVE_TO_POINT(setp_speed, setp_X, setp_Y) \
    do{ \
        command_t cmd; \
        cmd.type = RUN_TO_POINT; \
        cmd.speed = setp_speed; \
        cmd.point.x_pos = setp_X; \
        cmd.point.y_pos = setp_Y; \
        command_add(cmd); \
    }while(0)


#define ROBOT_ROTATE(setp_speed, setp_angle) \
    do{ \
        command_t cmd; \
        cmd.type = ROTATE; \
        cmd.speed = setp_speed; \
        cmd.angle = setp_angle; \
        command_add(cmd); \
    }while(0)


#define ROBOT_WAIT(setp_time) \
    do{ \
        command_t cmd; \
        cmd.type = WAIT_TIME; \
        cmd.time = setp_time; \
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
