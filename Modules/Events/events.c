#include "events.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <string.h>
#include "stdint.h"

static TaskHandle_t xHandlingTask;

typedef struct
{
    event_t event_value;
    event_callback callback;
    bool enabled;
}event_handler_t;


static event_handler_t events_handler_map[EVENT_MAX];

void Event_Register(event_t ev, event_callback callback){
    if (ev >= EVENT_MAX){
        return;
    }
    events_handler_map[ev].event_value = 1<<ev;
    events_handler_map[ev].callback = callback;
    events_handler_map[ev].enabled = true;
}

void Event_Unregister(event_t ev){
    if (ev >= EVENT_MAX){
        return;
    }
    events_handler_map[ev].event_value = 1<<ev;
    events_handler_map[ev].callback = NULL;
    events_handler_map[ev].enabled = false;
}

void Event_Notif(event_t ev){
    if (ev >= EVENT_MAX || xHandlingTask == NULL){
        return;
    }

    if (events_handler_map[ev].enabled){

        xTaskNotify( xHandlingTask, 1<<ev, eSetBits);
    }
}

void Event_Handle(){

    BaseType_t xResult;
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 10 );
    uint32_t event_received = 0;


    xResult = xTaskNotifyWait( pdFALSE,          /* Don't clear bits on entry. */
                                 UINT32_MAX,        /* Clear all bits on exit. */
                                 &event_received, /* Stores the notified value. */
                                 xMaxBlockTime );

    if( xResult == pdPASS )
    {
        for (uint32_t ev_idx = 0; ev_idx < EVENT_MAX; ev_idx++){
            if (event_received & events_handler_map[ev_idx].event_value != 0){
                events_handler_map[ev_idx].callback(NULL, 0);
            }
        }
    }
}

void Event_Task_Register(TaskHandle_t handle){
    xHandlingTask = handle;
}

