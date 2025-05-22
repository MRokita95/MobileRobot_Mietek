#include "robot.h"
#include "rob_reaction.h"

Mobile_Platform_t* m_robot;

void RobotReact_RegisterInstance(Mobile_Platform_t* robot){
    m_robot = robot;
}

void RobotReact_Stop(void){
    Robot_Stop(m_robot);
}