#ifndef MOBILE_PLATFORM_H_
#define MOBILE_PLATFORM_H_

#include <stdint.h>
#include <stdbool.h>


#define WHEEL_DIAMATER 65.f //[mm] 
#define PI 3.14f

typedef struct {
    bool mode_on;
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



#endif