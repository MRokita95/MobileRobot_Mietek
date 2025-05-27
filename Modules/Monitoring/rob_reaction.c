#include "robot.h"
#include "rob_reaction.h"
#include "commands.h"

Mobile_Platform_t* m_robot;

void RobotReact_RegisterInstance(Mobile_Platform_t* robot){
    m_robot = robot;
}

void RobotReact_Stop(){
    command_pcb_t command;
    command.apid = ROBOT_APP_ID;
    command.guard = Robot_Ready;
    command.dispatcher = Robot_Dispatch;
    command.payload.robcmd.robot = m_robot;
    command.payload.robcmd.type = STOP_ROB;
    Command_New(command, CRITICAL_SEVERITY);
}

void RobotReact_StopAndSafeReturn(){
    command_pcb_t command;
    command.apid = ROBOT_APP_ID;
    command.guard = Robot_Ready;
    command.dispatcher = Robot_Dispatch;
    command.payload.robcmd.robot = m_robot;
    command.payload.robcmd.type = STOP_ROB;
    command.timeout = 1000;
    Command_New(command, CRITICAL_SEVERITY);


    command.payload.robcmd.type = SAFE_RETURN;
    command.timeout = 50000;
    Command_New(command, CRITICAL_SEVERITY);
}