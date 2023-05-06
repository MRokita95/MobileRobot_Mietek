#include "commands.h"

typedef struct {
    int16_t write_idx;
    int16_t read_idx;
    command_t* current_cmd;
    command_buff_status_t buff_status;
} ringbuff_cmd_t;

static command_t m_commands[MAX_COMMANDS_CNT];

static ringbuff_cmd_t m_ringbuff_cmd = {
    .read_idx = -1,
    .write_idx = -1,
    .current_cmd = NULL,
    .buff_status = BUFF_EMPTY,
};


command_buff_status_t command_add(command_t command){
    command_t *cmd = m_commands;

    m_ringbuff_cmd.buff_status = BUFF_OK;

    if (m_ringbuff_cmd.write_idx == -1){
        m_ringbuff_cmd.write_idx = 0;
    }

    cmd += m_ringbuff_cmd.write_idx;

    if (cmd->status == IN_PROGRESS) {
        m_ringbuff_cmd.buff_status = BUFF_FULL;
        return m_ringbuff_cmd.buff_status;
    }

    *cmd = command;
    cmd->id = m_ringbuff_cmd.write_idx;
    cmd->status = IDLE;

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
    command_t *cmd = m_commands;

    if (m_ringbuff_cmd.read_idx == -1){
        m_ringbuff_cmd.read_idx = 0;
    }

    cmd += m_ringbuff_cmd.read_idx;

    if (cmd->status != IDLE){
        m_ringbuff_cmd.buff_status = BUFF_FULL;
        return BUFF_FULL;
    }

    m_ringbuff_cmd.read_idx++;
    if (m_ringbuff_cmd.read_idx >= MAX_COMMANDS_CNT){
        m_ringbuff_cmd.read_idx = -1;
    }

    m_ringbuff_cmd.current_cmd = cmd;

    next_command = cmd;
    
    if (m_ringbuff_cmd.read_idx == m_ringbuff_cmd.write_idx){
        m_ringbuff_cmd.buff_status = BUFF_EMPTY;
    }

    return m_ringbuff_cmd.buff_status;
}

void command_set_status(command_status_t status){
    command_t *cmd = m_commands;

    cmd += m_ringbuff_cmd.read_idx;
    cmd->status = status;
}

command_status_t command_actual_status(){
    command_t *cmd = m_commands;

    cmd += m_ringbuff_cmd.read_idx;
    return cmd->status;
}

command_buff_status_t command_buff_status(){
    return m_ringbuff_cmd.buff_status;
}
