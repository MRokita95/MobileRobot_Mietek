#ifndef TYPES_H_
#define TYPES_H_
#include "stdint.h"

typedef enum{
    EMPTY = 0,
    IDLE,
    QUEUED,
    READY,
    IN_PROGRESS,
    DONE_OK,
    ERR,
    TIMEOUT,
    INTERRUPTED
} command_status_t;

typedef void (*status_notif_cb)(command_status_t status, int32_t retval);

#endif