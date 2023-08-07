#include "commands.h"

typedef struct {
    int16_t write_idx;
    int16_t read_idx;
    command_t* current_cmd;
    command_buff_status_t buff_status;
} ringbuff_cmd_t;

static command_t* m_commands[MAX_COMMANDS_CNT];

static ringbuff_cmd_t m_ringbuff_cmd = {
    .read_idx = -1,
    .write_idx = -1,
    .current_cmd = NULL,
    .buff_status = BUFF_EMPTY,
};


command_buff_status_t command_add(command_t command){
    command_t **cmd = m_commands;

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
    *cmd = malloc(sizeof(command_t));
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

command_buff_status_t command_get_next(command_t* next_command){
    //command_t **cmd = m_commands;

    if (m_ringbuff_cmd.read_idx == -1 && m_ringbuff_cmd.write_idx == -1){
        return BUFF_EMPTY;
    }

    if (m_ringbuff_cmd.read_idx == m_ringbuff_cmd.write_idx){
            m_ringbuff_cmd.buff_status = BUFF_EMPTY;
        }

    if (m_ringbuff_cmd.read_idx == -1){
        m_ringbuff_cmd.read_idx = 0;
    }

    command_t **cmd  = &m_commands[m_ringbuff_cmd.read_idx];

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

void command_set_status(command_status_t status){
    if (m_ringbuff_cmd.current_cmd == NULL){
        return;
    }
	m_ringbuff_cmd.current_cmd->status = status;

    /*free the alocated memory for the executed command*/
    if (status == DONE_OK){
        free(m_ringbuff_cmd.current_cmd);
    }
}

command_status_t command_actual_status(){
    if (m_ringbuff_cmd.current_cmd == NULL){
        return EMPTY;
    }
    return m_ringbuff_cmd.current_cmd->status;
}

command_buff_status_t command_buff_status(){
    if (m_ringbuff_cmd.current_cmd == NULL){
        return BUFF_EMPTY;
    }
    return m_ringbuff_cmd.buff_status;
}
