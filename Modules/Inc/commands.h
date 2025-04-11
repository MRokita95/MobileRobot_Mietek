#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "robot.h"
#include "application_defs.h"

#include <stdint.h>

#define MAX_COMMANDS_CNT 100u

typedef enum{
    EMPTY = 0,
    IDLE,
    QUEUED,
    IN_PROGRESS,
    DONE_OK,
    ERR,
    TIMEOUT
} command_status_t;

typedef enum{
    READY,
    BLOCKED,
} command_cond_t;


typedef bool (*guard_cb)(payload_t* arg);
typedef void (*dispatcher_cb)(payload_t* arg);

typedef struct {
    uint16_t id;
    command_status_t status;
    command_cond_t cond;
    uint8_t apid;
    dispatcher_cb dispatcher;
    guard_cb guard;
    int32_t retval;
    bool background;    //TODO: implement
    payload_t payload;
} command_t;

typedef enum{
    BUFF_EMPTY = 0,
    BUFF_OK,
    BUFF_FULL,
    BUFF_NOK,
} command_buff_status_t;


command_buff_status_t command_add(command_t command);

command_buff_status_t command_get_next(command_t* command);

command_buff_status_t command_buff_status();

command_cond_t command_get_next_cond();

command_status_t command_actual_status(void);

void command_release(void);

void command_set_next_ready();

void command_set_status(command_status_t status);

uint16_t command_get_count(void);

void Management_Task(void);


#endif
