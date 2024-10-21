#ifndef STATE_HANDLING_H_
#define STATE_HANDLING_H_

#include "robot.h"

typedef enum{
    PROGRAMABLE,
    MANUAL,
    AUTONOMOUS,
}state_enum;

typedef void (*state_callback)(void *arg);

typedef struct{
    state_enum state;
    state_callback active_cb;
    state_callback enter_transition;
    state_callback end_transition;
}state_handle_t;


void State_Execute(void);

void State_Next(state_enum next_state);

void Register_State(state_handle_t* handle);

#endif
