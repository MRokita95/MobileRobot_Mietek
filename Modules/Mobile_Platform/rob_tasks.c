#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "robot.h"
#include "mobile_platform.h"
#include "commands.h"
#include "application_defs.h"

static command_pcb_t current_task;

static status_notif_cb status_notif;

static void exec_type(Mobile_Platform_t *robot, robot_command_t* cmd){

    switch (cmd->type)
    {
    case START_ROB:
        ROB_DEBUG("START ROB...\r\n");
        break;

    case AUTOPATH_START:
        ROB_DEBUG("AUTO ROB CMD...\r\n");
        Robot_SetMode(robot, AUTONOMOUS_MODE);
        break;

    case MANUAL_START:
        ROB_DEBUG("MANU ROB CMD...\r\n");
        Robot_SetMode(robot, MANUAL_MODE);
        break;

    case STOP_ROB:
        ROB_DEBUG("STOP/MANU/AUTO ROB CMD...\r\n");
        Robot_Stop(robot);
        break;

    case RUN_FOR_TIME:
        ROB_DEBUG("RUN FOR TIME CMD...\r\n");
        Robot_SetSpeed(robot, cmd->speed);
        Robot_StartTimer(robot, cmd->time);
        break;

    case RUN_FOR_DIST:
        ROB_DEBUG("RUN FOR DISTANCE CMD...\r\n");
        Robot_SetSpeed(robot, cmd->speed);
        Robot_SetDistance(robot, cmd->distance);
        break;

    case RUN_TO_POINT:
        ROB_DEBUG("RUN TO THE POINT...\r\n");
        Robot_MoveToPoint(robot, cmd->speed, cmd->point.x_pos, cmd->point.y_pos);
        break;

    case ROTATE:
        ROB_DEBUG("ROTATE...\r\n");
        Robot_Rotate(robot, cmd->speed, cmd->angle);
        break;

    case WAIT_TIME:
        ROB_DEBUG("WAIT ROB...\r\n");
        Robot_SetSpeed(robot, 0);
        Robot_StartTimer(robot, cmd->time);
        break;
    
    default:
        ROB_DEBUG("UNDEFINED COMMAND...\r\n");
        break;
    }
}


void End_Command_Execution(Mobile_Platform_t *robot, command_status_t status){

    status_notif(status, 0);
}

bool Robot_Ready(robot_payload_t* data){
    return Robot_Status(data->robcmd.robot) != ROB_IN_PROGRESS;
}

void Robot_Dispatch(robot_payload_t* data, status_notif_cb cb){

    status_notif = cb;
    status_notif(IN_PROGRESS, 0);

    exec_type(data->robcmd.robot, &data->robcmd);
}
