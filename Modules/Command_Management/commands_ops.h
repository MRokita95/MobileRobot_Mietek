#ifndef COMMANDS_OPS_H_
#define COMMANDS_OPS_H_

#include "commands.h"

#define MAX_NORM_COMMANDS_CNT 100u
#define MAX_HIGH_COMMANDS_CNT 10u
#define MAX_CRIT_COMMANDS_CNT 5u

typedef void (*queue_notif_cb)(void);

void command_buff_init();

command_buff_status_t command_get_next(command_pcb_t** command, command_severity_t severity);

command_buff_status_t command_buff_status();

command_status_t command_get_status(command_pcb_t* command);

void command_release(command_pcb_t* command);

void command_set_next_ready();

void command_set_status(command_pcb_t* command, command_status_t status);

uint16_t command_get_count(command_severity_t severity_queue);

void command_add_incoming_notif(command_severity_t severity, queue_notif_cb cb);  

#endif