#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include "timer.h"


uint32_t Time_Delta(uint32_t* reftime_ms, bool update_ref){
    uint32_t now;

    now = Hal_GetTick(); 
    uint32_t delta = now - (*reftime_ms/portTICK_PERIOD_MS);

    if (update_ref){
        *reftime_ms = now / portTICK_PERIOD_MS;
    }

    return delta;
}


void Timer_Start(Timer_t *timer, uint32_t timeout) {
    
    timer->working = true;
    timer->expired = false;
    timer->timeout = timeout;

    timer->start = Hal_GetTick()/portTICK_PERIOD_MS;
}


void Timer_Status(Timer_t *timer) {

    if (!timer->working){
        return;
    }
    uint32_t delta_ms =  Time_Delta(&timer->start, false);

    if (delta_ms >= timer->timeout){
        timer->working = false;
        timer->expired = true;
    }
}