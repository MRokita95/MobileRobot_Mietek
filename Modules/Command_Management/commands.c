#include "commands.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

typedef struct {
    int16_t write_idx;
    int16_t read_idx;
    command_pcb_t* current_cmd;
    command_buff_status_t buff_status;
    SemaphoreHandle_t status_access;
} ringbuff_cmd_t;

static command_pcb_t* m_commands[MAX_COMMANDS_CNT];

static ringbuff_cmd_t m_ringbuff_cmd = {
    .read_idx = -1,
    .write_idx = -1,
    .current_cmd = NULL,
    .buff_status = BUFF_EMPTY,
};


command_buff_status_t command_add(command_pcb_t command){
    command_pcb_t **cmd = m_commands;

    m_ringbuff_cmd.buff_status = BUFF_OK;

    if (m_ringbuff_cmd.write_idx == -1){
        m_ringbuff_cmd.write_idx = 0;
    }

    cmd += m_ringbuff_cmd.write_idx;

    if ((*cmd)->status == IN_PROGRESS && *cmd != NULL) {
        m_ringbuff_cmd.buff_status = BUFF_FULL;
        return m_ringbuff_cmd.buff_status;
    }

    /*allocate memory for the command*/
    *cmd = malloc(sizeof(command_pcb_t));
    if (cmd == NULL){
        return BUFF_NOK;
    }
    **cmd = command;
    (*cmd)->id = m_ringbuff_cmd.write_idx;
    (*cmd)->status = IDLE;

    m_ringbuff_cmd.write_idx++;
    if (m_ringbuff_cmd.write_idx >= MAX_COMMANDS_CNT){
        m_ringbuff_cmd.write_idx = -1;
    }

    if (m_ringbuff_cmd.write_idx == m_ringbuff_cmd.read_idx){
        m_ringbuff_cmd.buff_status = BUFF_FULL;
    }

    return m_ringbuff_cmd.buff_status;
}

command_buff_status_t command_get_next(command_pcb_t* next_command){
    //command_pcb_t **cmd = m_commands;

    if (m_ringbuff_cmd.read_idx == -1 && m_ringbuff_cmd.write_idx == -1){
        return BUFF_EMPTY;
    }

    if (m_ringbuff_cmd.read_idx == m_ringbuff_cmd.write_idx){
            m_ringbuff_cmd.buff_status = BUFF_EMPTY;
        }

    if (m_ringbuff_cmd.read_idx == -1){
        m_ringbuff_cmd.read_idx = 0;
    }

    command_pcb_t **cmd  = &m_commands[m_ringbuff_cmd.read_idx];

    if ((*cmd)->status == EMPTY || *cmd == NULL) {
        return BUFF_EMPTY;
    }
    else if ((*cmd)->status != IDLE){
        m_ringbuff_cmd.buff_status = BUFF_FULL;
        return BUFF_FULL;
    }

    m_ringbuff_cmd.read_idx++;
    if (m_ringbuff_cmd.read_idx >= MAX_COMMANDS_CNT){
        m_ringbuff_cmd.read_idx = -1;
    }

    m_ringbuff_cmd.current_cmd = *cmd;

    *next_command = **cmd;

    return m_ringbuff_cmd.buff_status;
}

void command_set_status(command_pcb_t* command, command_status_t status){
    if( xSemaphoreTake(m_ringbuff_cmd.status_access, portMAX_DELAY) == pdPASS){
	    command->status = status;

        xSemaphoreGive(m_ringbuff_cmd.status_access);
    }
}

void command_release(command_pcb_t* command){
    /*free the alocated memory for the executed command*/
    free(command);
}

command_status_t command_get_status(command_pcb_t* command){
    command_status_t sts;
    if( xSemaphoreTake(m_ringbuff_cmd.status_access, portMAX_DELAY) == pdPASS){
        sts = command->status;
        xSemaphoreGive(m_ringbuff_cmd.status_access);
    }
    return sts;
}

command_buff_status_t command_buff_status(){
    if (m_ringbuff_cmd.current_cmd == NULL && (m_ringbuff_cmd.read_idx == m_ringbuff_cmd.write_idx)){
        return BUFF_EMPTY;
    }
    return m_ringbuff_cmd.buff_status;
}


void command_set_next_ready(){
    if (m_ringbuff_cmd.read_idx == -1){
        return;
    }
    command_pcb_t **cmd  = &m_commands[m_ringbuff_cmd.read_idx];
    if ((*cmd == NULL) || ((*cmd)->status != IDLE) ){
        return;
    }
    command_set_status(*cmd, READY);
}

uint16_t command_get_count(){

    int32_t read_idx = (m_ringbuff_cmd.read_idx != -1) ? m_ringbuff_cmd.read_idx : 0;
    int32_t write_idx = (m_ringbuff_cmd.write_idx != -1) ? m_ringbuff_cmd.write_idx : 0;

    if (write_idx > read_idx){
        return write_idx - read_idx;
    }
    else if (write_idx == read_idx){
        return 0;
    }
    else {
        return MAX_COMMANDS_CNT - read_idx + write_idx;
    }
}

void command_buff_init(){

    m_ringbuff_cmd.status_access = xSemaphoreCreateMutex();
}
