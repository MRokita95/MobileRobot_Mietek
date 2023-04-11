#ifndef ROBOT_APP_H_
#define ROBOT_APP_H_


extern Mobile_Platform_t robot;

/**
 * @brief file contains Robot macros (functions wrappers) for application creation
 * 
 */

#define ROBOT_MOVE_DISTANCE(distance, speed) \
    do{ \
        Robot_SetPath(robot, speed, 0); \
        Robot_SetDistance(robot, distance); \
    }while(0);


#define ROBOT_MOVE_SPEED(speed, angle) \
    do{ \
        Robot_SetPath(robot, speed, angle); \
    }while(0);


#define ROBOT_MOVE_SPEED_TIME(speed, angle, timeout) \
    do{ \
        Robot_SetPath(robot, speed, angle); \
        Robot_StartTimer(robot, timeout) \
    }while(0);


#endif