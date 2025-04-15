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
        command_pcb_t cmd; \
        cmd.payload.robcmd.type = RUN_FOR_DIST; \
        cmd.payload.robcmd.distance = setp_distance; \
        cmd.payload.robcmd.speed = setp_speed; \
        cmd.payload.robcmd.robot = robot; \
        cmd.payload.robcmd.guard = Robot_Ready; \
        cmd.payload.robcmd.dispatch = Robot_Dispatch; \
        command_add(cmd); \
    }while(0)


#define ROBOT_MOVE_SPEED(setp_speed, setp_time) \
    do{ \
        command_pcb_t cmd; \
        cmd.payload.robcmd.type = RUN_FOR_TIME; \
        cmd.payload.robcmd.speed = setp_speed; \
        cmd.payload.robcmd.time = setp_time; \
        cmd.payload.robcmd.robot = &robot; \
        cmd.guard = Robot_Ready; \
        cmd.dispatch = Robot_Dispatch; \
        command_add(cmd); \
    }while(0)


#define ROBOT_MOVE_TO_POINT(setp_speed, setp_X, setp_Y) \
    do{ \
        command_pcb_t cmd; \
        cmd.payload.robcmd.type = RUN_TO_POINT; \
        cmd.payload.robcmd.speed = setp_speed; \
        cmd.payload.robcmd.point.x_pos = setp_X; \
        cmd.payload.robcmd.point.y_pos = setp_Y; \
        cmd.payload.robcmd.robot = &robot; \
        cmd.guard = Robot_Ready; \
        cmd.dispatcher = Robot_Dispatch; \
        command_add(cmd); \
    }while(0)


#define ROBOT_ROTATE(setp_speed, setp_angle) \
    do{ \
        command_pcb_t cmd; \
        cmd.payload.robcmd.type = ROTATE; \
        cmd.payload.robcmd.speed = setp_speed; \
        cmd.payload.robcmd.angle = setp_angle; \
        cmd.payload.robcmd.robot = &robot; \
        cmd.guard = Robot_Ready; \
        cmd.dispatcher = Robot_Dispatch; \
        command_add(cmd); \
    }while(0)


#define ROBOT_WAIT(setp_time) \
    do{ \
        command_pcb_t cmd; \
        cmd.payload.robcmd.type = WAIT_TIME; \
        cmd.payload.robcmd.time = setp_time; \
        cmd.payload.robcmd.robot = &robot; \
        cmd.guard = Robot_Ready; \
        cmd.dispatcher = Robot_Dispatch; \
        command_add(cmd); \
    }while(0)


#define ROBOT_ON() \
    do{ \
        command_pcb_t cmd; \
        cmd.payload.robcmd.type = START_ROB; \
        cmd.payload.robcmd.robot = &robot; \
        cmd.guard = Robot_Ready; \
        cmd.dispatcher = Robot_Dispatch; \
        command_add(cmd); \
    }while(0)


#define ROBOT_OFF() \
    do{ \
        command_pcb_t cmd; \
        cmd.payload.robcmd.type = STOP_ROB; \
        cmd.payload.robcmd.robot = &robot; \
        cmd.guard = Robot_Ready; \
        cmd.dispatcher = Robot_Dispatch; \
        command_add(cmd); \
    }while(0)

#endif
