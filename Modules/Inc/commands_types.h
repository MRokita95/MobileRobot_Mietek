#ifndef TYPES_H_
#define TYPES_H_

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

typedef void (*status_update_cb)(command_status_t status);

#endif