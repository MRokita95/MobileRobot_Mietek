#include "rob_monitoring.h"
#include "sensors_common.h"
#include "robot.h"


#define CRITICAL_WHEEL_SPEED (2)
#define CRITICAL_DIFF_FACTOR (10)


Mobile_Platform_t* m_robot;


void RobotMon_RegisterInstance(Mobile_Platform_t* robot){
    m_robot = robot;
}

bool RobotMon_OnMovement(void){
    if (Robot_Status(m_robot) != ROB_IN_PROGRESS){
        return false;
    }
    robot_mode_t mode = Robot_ActiveMode(m_robot);
    bool on_moving_mode = (mode == TIMER_MODE && m_robot->speed_setpoint > 0) || 
                        (mode == POINT_MODE) || 
                        (mode == ORIENT_MODE) || 
                        (mode == DISTANCE_MODE);
    return on_moving_mode;
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
    } else if (abs_left_wheel_speed < CRITICAL_WHEEL_SPEED && abs_right_wheel_speed < CRITICAL_WHEEL_SPEED){
        return true;
    }
    return false;
}


bool ImuMon_ActiveCheck(){
    return Sensor_GetState(IMU) == SENSOR_WORKING;
}

union value ImuMon_GetAngle(uint8_t param){
    union value value = {.f_value = 0};
    euler_angles_t angle;
    Sensor_GetValue(IMU, &angle);
    if (param == 0){
        //ROLL
        value.f_value = angle.roll;
    } else if (param == 1){
        //PITCH
        value.f_value = angle.pitch;
    }
    return value;
}

