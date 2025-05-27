#ifndef MONITORING_H_
#define MONITORING_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"

#include "robot.h"

#define MAX_CONDITIONS_CNT 2u

typedef enum{
    UINT8_CHECK,
    UINT32_CHECK,
    INT32_CHECK,
    FLOAT_CHECK,
}check_type_t;

typedef enum{
    ROBOT_DEVICE,
    IMU_DEVICE,
} monitored_devices_t;

typedef enum{
    MON_ENABLED,
    MON_DISABLED
} monitoring_state_t;

union value
{
    uint8_t u8_value;
    uint32_t u32_value;
    int32_t i32_value;
    float f_value;
};

typedef bool (*action_cb)(void);
typedef bool (*conditional_cb)(void);
typedef void (*reaction_cb)(void);
typedef union value (*get_value_cb)(uint8_t);

typedef struct{
    monitoring_state_t state;
    reaction_cb reaction;
    uint8_t samples;
    uint8_t interval;
    check_type_t type;
    conditional_cb condition[MAX_CONDITIONS_CNT];
    get_value_cb getval;
    uint8_t param_id;
    
    union value current_value;
    union value prev_value[2];
    union value avg_value;

    union value high_limit;
    union value low_limit;
}check_param_t;

typedef struct{
    monitoring_state_t state;
    reaction_cb reaction;
    conditional_cb condition[MAX_CONDITIONS_CNT];
    uint8_t interval;
    uint8_t samples;
    action_cb action;
    void* dev_instance;
    monitored_devices_t dev_id;
}check_action_t;


void Monitoring_Init(void);

void Monitorig_RegisterRobot(Mobile_Platform_t* robot);

void Monitoring_Execute();

void Monitoring_Setup(monitoring_state_t state);

#endif
