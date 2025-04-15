#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "robot.h"
#include "application_defs.h"
#include "commands_types.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#include <stdint.h>

#define MAX_COMMANDS_CNT 100u


typedef bool (*guard_cb)(payload_t* data);
typedef void (*dispatcher_cb)(payload_t* data, status_update_cb cb);

typedef struct {
    uint16_t id;
    command_status_t status;
    uint8_t apid;
    dispatcher_cb dispatcher;
    guard_cb guard;
    int32_t retval;
    bool background;    //TODO: implement
    payload_t payload;
} command_pcb_t;

typedef enum{
    BUFF_EMPTY = 0,
    BUFF_OK,
    BUFF_FULL,
    BUFF_NOK,
} command_buff_status_t;

void command_buff_init();

command_buff_status_t command_add(command_pcb_t command);

command_buff_status_t command_get_next(command_pcb_t* command);

command_buff_status_t command_buff_status();

command_status_t command_get_status(command_pcb_t* command);

void command_release(command_pcb_t* command);

void command_set_next_ready();

void command_set_status(command_pcb_t* command, command_status_t status);

uint16_t command_get_count(void);

void Management_Task(void);

void Commands_Scheduler(void);

void Commands_Scheduler_Resume(void);

void Commands_Scheduler_Init(TaskHandle_t xHandle);


#endif
