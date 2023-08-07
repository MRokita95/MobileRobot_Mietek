#ifndef TASK_WORKERS_H_
#define TASK_WORKERS_H_



#define SENSOR_TASK_FREQUENCY 100U
#define COMM_TASK_FREQUENCY 500U
#define ROB_TASK_FREQUENCY 100U
#define MANAG_TASK_FREQUENCY 1000U


#define TASK_NUMBERS 5u


#define COMM_TASK 1u
#define SENSOR_TASK 2u
#define MANAGE_TASK 3u
#define ROBOT_TASK 4u


#define TIMER_NUMBERS 0u


void TasksWorkers_Init(void);

void Robot_Application1(void);

#endif