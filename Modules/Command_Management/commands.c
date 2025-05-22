#include "commands.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "commands_ops.h"

typedef struct {
    int16_t write_idx;
    int16_t read_idx;
    int16_t max_idx;
    command_pcb_t* queue;
    command_pcb_t* current_cmd;
    command_buff_status_t buff_status;
    SemaphoreHandle_t status_access;
    queue_notif_cb incoming_notif;
} ringbuff_cmd_t;

static command_pcb_t m_normal_commands[MAX_NORM_COMMANDS_CNT];
static command_pcb_t m_high_commands[MAX_HIGH_COMMANDS_CNT];
static command_pcb_t m_crit_commands[MAX_CRIT_COMMANDS_CNT];

static ringbuff_cmd_t m_ringbuff_cmd[MAX_SEVERITY] = 
{ 
    [NORMAL_SEVERITY] = 
    {
        .read_idx = -1,
        .write_idx = -1,
        .max_idx = MAX_NORM_COMMANDS_CNT,
        .current_cmd = NULL,
        .buff_status = BUFF_EMPTY,
        .queue = m_normal_commands
    },
    [HIGH_SEVERITY] = 
    {
        .read_idx = -1,
        .write_idx = -1,
        .max_idx = MAX_HIGH_COMMANDS_CNT,
        .current_cmd = NULL,
        .buff_status = BUFF_EMPTY,
        .queue = m_high_commands
    },
    [CRITICAL_SEVERITY] = 
    {
        .read_idx = -1,
        .write_idx = -1,
        .max_idx = MAX_CRIT_COMMANDS_CNT,
        .current_cmd = NULL,
        .buff_status = BUFF_EMPTY,
        .queue = m_crit_commands
    },
};

void Command_ResetQueue(command_severity_t severity){
    if (severity >= MAX_SEVERITY){
        return BUFF_NOK;
    }

    ringbuff_cmd_t* ringbuff = &m_ringbuff_cmd[severity];

    if (ringbuff->read_idx == -1){
        ringbuff->read_idx = 0;
    }

    if (ringbuff->read_idx > ringbuff->write_idx){
        for (uint16_t idx = ringbuff->read_idx; idx < ringbuff->max_idx; idx++){
            ringbuff->queue[idx].status = EMPTY;
        }
        ringbuff->read_idx = 0;
    }

    if (ringbuff->read_idx < ringbuff->write_idx){
        for (; ringbuff->read_idx < ringbuff->write_idx; ringbuff->read_idx++){
            ringbuff->queue[ringbuff->read_idx].status = EMPTY;
        }
    }

}

command_buff_status_t Command_New(command_pcb_t command, command_severity_t severity){

    if (severity >= MAX_SEVERITY){
        return BUFF_NOK;
    }

    ringbuff_cmd_t* ringbuff = &m_ringbuff_cmd[severity];

    ringbuff->buff_status = BUFF_OK;

    if (ringbuff->write_idx == -1){
        ringbuff->write_idx = 0;
    }

    command_pcb_t *cmd = &ringbuff->queue[ringbuff->write_idx];

    if ((cmd)->status == IN_PROGRESS && cmd != NULL) {
        ringbuff->buff_status = BUFF_FULL;
        return ringbuff->buff_status;
    }

    if (cmd == NULL){
        return BUFF_NOK;
    }
    *cmd = command;
    (cmd)->severity = severity;
    (cmd)->id = ringbuff->write_idx;
    (cmd)->status = IDLE;

    if (ringbuff->incoming_notif != NULL){
        ringbuff->incoming_notif();
    }

    ringbuff->write_idx++;
    if (ringbuff->write_idx >= ringbuff->max_idx){
        ringbuff->write_idx = -1;
    }

    if (ringbuff->write_idx == ringbuff->read_idx){
        ringbuff->buff_status = BUFF_FULL;
    }

    return ringbuff->buff_status;
}

command_buff_status_t command_get_next(command_pcb_t** next_command, command_severity_t severity){
    if (severity >= MAX_SEVERITY){
        return BUFF_NOK;
    }

    ringbuff_cmd_t* ringbuff = &m_ringbuff_cmd[severity];

    if (ringbuff->read_idx == -1 && ringbuff->write_idx == -1){
        return BUFF_EMPTY;
    }

    if (ringbuff->read_idx == ringbuff->write_idx){
            ringbuff->buff_status = BUFF_EMPTY;
        }

    if (ringbuff->read_idx == -1){
        ringbuff->read_idx = 0;
    }

    command_pcb_t *cmd  = &ringbuff->queue[ringbuff->read_idx];

    if ((cmd)->status == EMPTY || cmd == NULL) {
        return BUFF_EMPTY;
    }
    else if ((cmd)->status != IDLE){
        ringbuff->buff_status = BUFF_FULL;
        return BUFF_FULL;
    }

    *next_command = cmd;

    ringbuff->read_idx++;
    if (ringbuff->read_idx >= ringbuff->max_idx){
        ringbuff->read_idx = -1;
    }

    ringbuff->current_cmd = cmd;

    return ringbuff->buff_status;
}

void command_set_status(command_pcb_t* command, command_status_t status){

    const command_severity_t severity = NORMAL_SEVERITY;
    ringbuff_cmd_t* ringbuff = &m_ringbuff_cmd[severity];

    if( xSemaphoreTake(ringbuff->status_access, portMAX_DELAY) == pdPASS){
	    command->status = status;

        xSemaphoreGive(ringbuff->status_access);
    }
}

void command_release(command_pcb_t* command){
    /*free the alocated memory for the executed command*/
    //command_set_status(command, EMPTY);
}

command_status_t command_get_status(command_pcb_t* command){

    const command_severity_t severity = command->severity;
    ringbuff_cmd_t* ringbuff = &m_ringbuff_cmd[severity];

    command_status_t sts;
    if( xSemaphoreTake(ringbuff->status_access, portMAX_DELAY) == pdPASS){
        sts = command->status;
        xSemaphoreGive(ringbuff->status_access);
    }
    return sts;
}

command_buff_status_t command_buff_status(){

    const command_severity_t severity = NORMAL_SEVERITY;
    ringbuff_cmd_t* ringbuff = &m_ringbuff_cmd[severity];

    if (ringbuff->current_cmd == NULL && (ringbuff->read_idx == ringbuff->write_idx)){
        return BUFF_EMPTY;
    }
    return ringbuff->buff_status;
}


void command_set_next_ready(){

    const command_severity_t severity = NORMAL_SEVERITY;
    ringbuff_cmd_t* ringbuff = &m_ringbuff_cmd[severity];


    if (ringbuff->read_idx == -1){
        return;
    }
    command_pcb_t *cmd  = &ringbuff->queue[ringbuff->read_idx];
    if ((cmd == NULL) || ((cmd)->status != IDLE) ){
        return;
    }
    command_set_status(cmd, READY);
}

uint16_t command_get_count(command_severity_t severity){

    if (severity >= MAX_SEVERITY){
        return BUFF_NOK;
    }

    ringbuff_cmd_t* ringbuff = &m_ringbuff_cmd[severity];


    int32_t read_idx = (ringbuff->read_idx != -1) ? ringbuff->read_idx : 0;
    int32_t write_idx = (ringbuff->write_idx != -1) ? ringbuff->write_idx : 0;

    if (write_idx > read_idx){
        return write_idx - read_idx;
    }
    else if (write_idx == read_idx){
        return 0;
    }
    else {
        return ringbuff->max_idx - read_idx + write_idx;
    }
}

void command_add_incoming_notif(command_severity_t severity, queue_notif_cb cb) {
    m_ringbuff_cmd[severity].incoming_notif = cb;
}

void command_buff_init(){

    m_ringbuff_cmd[NORMAL_SEVERITY].status_access = xSemaphoreCreateMutex();
    m_ringbuff_cmd[HIGH_SEVERITY].status_access = xSemaphoreCreateMutex();
    m_ringbuff_cmd[CRITICAL_SEVERITY].status_access = xSemaphoreCreateMutex();
}
