#ifndef MOBILE_PLATFORM_H_
#define MOBILE_PLATFORM_H_

#include <stdint.h>
#include <stdbool.h>
#include "commands.h"

#define WHEEL_DIAMATER 65.f //[mm] 
#define PI 3.14f

#define ALIGN_SPEED 20

typedef struct {
    bool mode_on;
    bool preset;
    int32_t actual_distance;   //in [mm]
    int32_t setpoint_distance;   //in [mm]
    uint32_t start_time;
    uint32_t current_time;
} distance_mode_t;

typedef struct {
    bool mode_on;
    uint32_t start_time;
    uint32_t stop_time;
} time_mode_t;


typedef enum{
    TIMER = 0,
    DISTANCE,
    POINT,
} current_setpoint_t;


void HK_Init(Mobile_Platform_t* robot);

void HK_Update(Mobile_Platform_t* robot);

void HK_Setpoints(void* setpoint, current_setpoint_t setpoint_type);

bool Execute_Command();

void End_Command_Execution(command_status_t status);

#endif