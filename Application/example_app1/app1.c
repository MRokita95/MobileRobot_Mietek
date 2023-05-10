#include "robot_app.h"


void Robot_Application1(){
    ROBOT_MOVE_SPEED(100, 1000);
    ROBOT_WAIT(2000);
    ROBOT_MOVE_SPEED(-100, 1000);
    ROBOT_WAIT(2000);
}