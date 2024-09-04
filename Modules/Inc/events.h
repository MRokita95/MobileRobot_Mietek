#ifndef EVENTS_H_
#define EVENTS_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"

typedef enum{
    EVENT_ROBOT_STUCKED,
    EVENT_MOTOR_STOPPED,
    EVENT_ROLL_TO_HIGH,
    EVENT_YAW_TOO_HIGH,
    EVENT_MAX,
} event_t;


typedef void (*event_callback)(uint8_t*, uint16_t);

void Event_Register(event_t ev, event_callback callback);

void Event_Unregister(event_t ev);

void Event_Notif(event_t ev);

void Event_Handle();

void Event_Task_Register(TaskHandle_t handle);

#endif
