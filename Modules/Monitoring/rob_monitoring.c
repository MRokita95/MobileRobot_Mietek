#include "rob_monitoring.h"

#include "robot.h"


#define CRITICAL_WHEEL_SPEED (2)
#define CRITICAL_DIFF_FACTOR (10)


Mobile_Platform_t* m_robot;


void RobotMon_RegisterInstance(Mobile_Platform_t* robot){
    m_robot = robot;
}

bool RobotMon_Stuck_Condition(void){
    return Robot_Status(m_robot) == ROB_IN_PROGRESS;
}

bool RobotMon_Stuck_Check(void){
    int32_t abs_right_wheel_speed = Robot_GetWheelSpeed(m_robot, RIGHT);
    int32_t abs_left_wheel_speed = Robot_GetWheelSpeed(m_robot, LEFT);

    if ((abs_right_wheel_speed < CRITICAL_WHEEL_SPEED) && (abs_left_wheel_speed > (CRITICAL_WHEEL_SPEED * CRITICAL_DIFF_FACTOR))){

        return true;
    } else if (abs_right_wheel_speed < (abs_left_wheel_speed * CRITICAL_DIFF_FACTOR)){

        return true;
    } else if ((abs_left_wheel_speed < CRITICAL_WHEEL_SPEED) && (abs_right_wheel_speed > (CRITICAL_WHEEL_SPEED * CRITICAL_DIFF_FACTOR))){

        return true;
    } else if (abs_left_wheel_speed < (abs_right_wheel_speed * CRITICAL_DIFF_FACTOR)){

        return true;
    }

}

