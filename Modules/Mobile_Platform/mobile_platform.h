#ifndef MOBILE_PLATFORM_H_
#define MOBILE_PLATFORM_H_

#include <stdint.h>
#include <stdbool.h>
#include "commands.h"

#define WHEEL_DIAMATER 65.f //[mm] 
#define PI 3.14f
#define MOBILE_PLATFORM_WIDTH 168.f //MAX is 180 mm

#define ALIGN_SPEED 60
#define RAMP_DIST 20 //mm before setpoint distance

typedef struct {
    bool mode_on;
    bool preset;
    bool slow_ramp_on;
    float actual_distance;   //in [mm]
    float setpoint_distance;   //in [mm]
    float ramp_distance;
    uint32_t start_time;
    uint32_t current_time;
} distance_mode_t;

typedef struct {
    bool mode_on;
    uint32_t start_time;
    uint32_t stop_time;
} time_mode_t;


typedef struct {
    bool mode_on;
    float actual;
    float setpoint;
    bool clockwise_on;
} rob_angle_t;


typedef enum{
    TIMER = 0,
    DISTANCE,
    POINT,
} current_setpoint_t;


void HK_Init(Mobile_Platform_t* robot);

void HK_Update(Mobile_Platform_t* robot);

void HK_Setpoints(void* setpoint, current_setpoint_t setpoint_type);

float Robot_GetOrient(Mobile_Platform_t* robot);

void Robot_Rotate(Mobile_Platform_t* robot, int32_t speed, int16_t angle_setpoint);

bool Execute_Command();

void End_Command_Execution(command_status_t status);

#endif
