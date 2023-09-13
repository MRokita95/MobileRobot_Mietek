#ifndef TRACING_H_
#define TRACING_H_

#include <stdlib.h>
#include "sensors_common.h"
#include "robot.h"


#define TRACE_DATA_FREQUENCY     (5000u)

typedef enum{
    OK = 0,
    TRACE_BUFFER_FULL,
}trace_data_status_t;

typedef struct{
    uint32_t timestamp;
    rob_coord_t rob_pos;
    euler_angles_t rob_orient;
}trace_data_t;

/**
 * @brief 
 * 
 */
void Trace_InitAccessInstances(Mobile_Platform_t *robot);

/**
 * @brief Access data from the robot (or other sensors),
 * and put them into buffer.
 * @note Function accesses itself proper data 
 * 
 */
void Trace_PullData(void);


/**
 * @brief 
 * 
 */
void Trace_FlushData(trace_data_t *data, uint16_t *size);

#endif