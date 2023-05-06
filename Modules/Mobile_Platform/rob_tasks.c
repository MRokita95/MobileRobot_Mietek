#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "robot.h"
#include "mobile_platform.h"
#include "commands.h"

extern Mobile_Platform_t robot;

static command_t current_task;

static bool is_ended(command_t* cmd){
    return cmd->status != IN_PROGRESS ;
}

static void exec_type(command_t* cmd){

    switch (cmd->type)
    {
    case START_ROB:
        ROB_DEBUG("START ROB...\r\n");
        break;

    case STOP_ROB:
        ROB_DEBUG("STOP ROB CMD...\r\n");
        Robot_Stop(&robot);
        break;

    case RUN_FOR_TIME:
        ROB_DEBUG("RUN FOR TIME CMD...\r\n");
        Robot_SetSpeed(&robot, cmd->speed);
        Robot_StartTimer(&robot, cmd->time);
        break;

    case RUN_FOR_DIST:
        ROB_DEBUG("RUN FOR DISTANCE CMD...\r\n");
        Robot_SetSpeed(&robot, cmd->speed);
        Robot_SetDistance(&robot, cmd->distance);
        break;

    case RUN_TO_POINT:
        ROB_DEBUG("RUN TO THE POINT...\r\n");
        Robot_MoveToPoint(&robot, cmd->speed, cmd->point.x_pos, cmd->point.y_pos);
        break;

    case WAIT_TIME:
        ROB_DEBUG("WAIT ROB...\r\n");
        Robot_SetSpeed(&robot, 0);
        Robot_StartTimer(&robot, cmd->time);
        break;
    
    default:
        break;
    }
}


bool Execute_Command(){

    command_t* rob_task = &current_task;

    if (!is_ended(&current_task) || Robot_Status(&robot) == ROB_IN_PROGRESS){
        return false;
    }

    command_t cmd;
    command_buff_status_t buff_status = command_get_next(&cmd);

    if (buff_status == BUFF_EMPTY){
        return false;
    }

    *rob_task = cmd;
    rob_task->status = IN_PROGRESS;

    exec_type(rob_task);
}


void End_Command_Execution(command_status_t status){
    command_t* rob_task = &current_task;

    rob_task->status = status;
}
