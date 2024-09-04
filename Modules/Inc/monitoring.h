#ifndef MONITORING_H_
#define MONITORING_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "events.h"

typedef enum{
    UINT8_CHECK,
    UINT32_CHECK,
    INT32_CHECK,
    FLOAT_CHECK,
}check_type_t;

union value
{
    uint8_t u8_value;
    uint32_t u32_value;
    int32_t i32_value;
    float f_value;
};

typedef bool (*action_cb)(void);
typedef bool (*conditional_cb)(void);

typedef struct{
    bool active;
    uint8_t samples;
    check_type_t type;
    conditional_cb condition;
    
    union value current_value;
    union value prev_value;
    union value avg_value;

    union value high_limit;
    union value low_limit;

    event_t event_below_limit;
    event_t event_above_limit;
}check_param_t;

typedef struct{
    conditional_cb condition;
    action_cb action;
    event_t event_notif;
}check_action_t;

void Monitoring_Init(void);

void Monitoring_Execute();



#endif
