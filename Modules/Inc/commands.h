#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "robot.h"
#include "application_defs.h"
#include "commands_types.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#include <stdint.h>


typedef bool (*guard_cb)(payload_t* data);
typedef void (*dispatcher_cb)(payload_t* data, status_notif_cb cb);

typedef enum {
    NORMAL_SEVERITY,
    HIGH_SEVERITY,
    CRITICAL_SEVERITY,
    MAX_SEVERITY
} command_severity_t;

typedef struct {
    uint16_t id;
    command_status_t status;
    command_severity_t severity;
    uint8_t apid;
    dispatcher_cb dispatcher;
    guard_cb guard;
    int32_t retval;
    uint16_t timeout;   //TODO: implement
    bool background;    //TODO: implement???
    payload_t payload;
} command_pcb_t;

typedef enum{
    BUFF_EMPTY = 0,
    BUFF_OK,
    BUFF_FULL,
    BUFF_NOK,
} command_buff_status_t;



command_buff_status_t Command_New(command_pcb_t command, command_severity_t severity);

void Command_ResetQueue(command_severity_t severity);

void Management_Task(void);

void Commands_Scheduler(void);

void Commands_Scheduler_Resume(void);

void Commands_Scheduler_Init(TaskHandle_t xHandle);


#endif
