#ifndef ROB_MONITORING_H_
#define ROB_MONITORING_H_


#include "monitoring.h"
#include "robot.h"

void RobotMon_RegisterInstance(Mobile_Platform_t* robot);

bool RobotMon_Stuck_Condition(void);

bool RobotMon_Stuck_Check(void);

#endif