#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>
#include <stdbool.h>


typedef struct {
    uint32_t start;
    uint32_t timeout;
    bool working;
    bool expired;
} Timer_t;



uint32_t Time_Delta(uint32_t* reftime_ms, bool update_ref);

void Timer_Start(Timer_t *timer, uint32_t timeout);



#endif