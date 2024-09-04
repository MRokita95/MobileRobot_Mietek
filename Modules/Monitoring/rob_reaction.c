#include "robot.h"
#include "rob_reaction.h"

extern Mobile_Platform_t* robot;

void RobotReact_Stop(uint8_t* param, uint16_t size){
    Robot_Stop(robot);
}