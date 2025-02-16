#ifndef ROB_REACTION_H_
#define ROB_REACTION_H_


#include "monitoring.h"
#include "robot.h"

void RobotReact_RegisterInstance(Mobile_Platform_t* robot);

void RobotReact_Stop(uint8_t* param, uint16_t size);


#endif