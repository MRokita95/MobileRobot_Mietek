#include "commands.h"
#include "application_defs.h"

typedef enum{
    ST_INIT,
    ST_SELECTION,
    ST_START,
    ST_IN_PROGRESS,
    ST_FINISH,
    ST_MAX
}SchedEnum;

typedef SchedEnum (*state_handler_cb)(void);

typedef struct{
    SchedEnum state;
    SchedEnum next_state;
    state_handler_cb state_handler[ST_MAX] ;
    command_t running_cmd;
    uint16_t running_idx;
    uint16_t max_idx;
}SchedState_t;

static SchedEnum init_handler(void);
static SchedEnum select_handler(void);
static SchedEnum start_handler(void);
static SchedEnum running_handler(void);
static SchedEnum finish_handler(void);


static SchedState_t m_scheduler = {
    .state = ST_INIT,
    .state_handler[ST_INIT] = init_handler,
    .state_handler[ST_SELECTION] = select_handler,
    .state_handler[ST_START] = start_handler,
    .state_handler[ST_IN_PROGRESS] = running_handler,
    .state_handler[ST_FINISH] = finish_handler,
    .running_cmd = NULL,
};

static SchedEnum init_handler(){
    m_scheduler.max_idx = command_get_count();
    if (m_scheduler.max_idx == 0){
        return ST_INIT;
    }
    command_buff_status_t buff_status = command_get_next(&m_scheduler.running_cmd);
    if (buff_status == BUFF_EMPTY || m_scheduler.running_cmd.status == EMPTY){
        command_release();
        return ST_FINISH;
    }
    command_set_status(QUEUED);
    return ST_START;
}

static SchedEnum select_handler(){

    if (command_get_next_cond() == READY){
            
        command_buff_status_t buff_status = command_get_next(&m_scheduler.running_cmd);

        if (buff_status == BUFF_EMPTY || m_scheduler.running_cmd.status == EMPTY){
            return ST_FINISH;
        }
        command_set_status(QUEUED);
        return ST_START;
    }
    return ST_SELECTION;
}

static SchedEnum start_handler(){

    if (command_actual_status() == QUEUED){
        if (m_scheduler.running_cmd.guard(&m_scheduler.running_cmd.payload)){
            m_scheduler.running_cmd.dispatcher(&m_scheduler.running_cmd.payload);
            m_scheduler.running_idx++;
            return ST_IN_PROGRESS;
        }
        return ST_START;
    }
    return ST_FINISH;
}

static SchedEnum running_handler(){

    command_status_t sts = command_actual_status();
    if (sts == DONE_OK){
        return ST_FINISH;
    } else if (sts = ERR){
        return ST_FINISH;
    } else if (sts = TIMEOUT){
        return ST_FINISH;
    }
}

static SchedEnum finish_handler(){

    if (command_get_count() == 0){
        return ST_FINISH;   //infinite loop, but task will be blocked
    }

    if (m_scheduler.running_idx == m_scheduler.max_idx){
        m_scheduler.running_idx = 0;
        m_scheduler.max_idx = 0;
        return ST_INIT;
    }

    command_set_next_ready();
    command_release();
    return ST_SELECTION;
}

void Commands_Scheduler(){

    m_scheduler.next_state = m_scheduler.state_handler[m_scheduler.state]();

    m_scheduler.state = m_scheduler.next_state;
}
