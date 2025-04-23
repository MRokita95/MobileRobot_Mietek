#include "commands.h"
#include "commands_ops.h"
#include "application_defs.h"
#include "task.h"

//todo: change start to dispatch name

typedef enum{
    ST_INIT,
    ST_SELECTION,
    ST_DISPATCH,
    ST_IN_PROGRESS,
    ST_FINISH,
    ST_MAX
}SchedEnum;

typedef SchedEnum (*state_handler_cb)(void);

typedef struct{
    SchedEnum state;
    SchedEnum next_state;
    state_handler_cb state_handler[ST_MAX] ;
    command_pcb_t *running_cmd;
    uint16_t max_idx;
    TaskHandle_t taskHandle;
}SchedState_t;

static SchedEnum init_handler(void);
static SchedEnum select_handler(void);
static SchedEnum dispatch_handler(void);
static SchedEnum running_handler(void);
static SchedEnum finish_handler(void);
static void update_cmd_status(command_status_t status);

static SchedState_t m_scheduler = {
    .state = ST_INIT,
    .state_handler[ST_INIT] = init_handler,
    .state_handler[ST_SELECTION] = select_handler,
    .state_handler[ST_DISPATCH] = dispatch_handler,
    .state_handler[ST_IN_PROGRESS] = running_handler,
    .state_handler[ST_FINISH] = finish_handler,
    .running_cmd = NULL,
};

static SchedEnum init_handler(){
    m_scheduler.max_idx = command_get_count();
    if (m_scheduler.max_idx == 0){
        return ST_FINISH;
    }
    command_buff_status_t buff_status = command_get_next(&m_scheduler.running_cmd);
    if (buff_status == BUFF_EMPTY || command_get_status(m_scheduler.running_cmd) == EMPTY){
        command_release(m_scheduler.running_cmd);
        return ST_FINISH;
    }
    update_cmd_status(QUEUED);
    return ST_SELECTION;
}

static SchedEnum select_handler(){

    if (command_get_status(m_scheduler.running_cmd) == QUEUED){
        if (m_scheduler.running_cmd->guard(&m_scheduler.running_cmd->payload)){
            update_cmd_status(READY);
            return ST_DISPATCH;
        }
    }
    return ST_SELECTION;
}

static SchedEnum dispatch_handler(){

    if (command_get_status(m_scheduler.running_cmd) == READY){
        m_scheduler.running_cmd->dispatcher(&m_scheduler.running_cmd->payload, update_cmd_status);
        return ST_IN_PROGRESS;
    }
    return ST_FINISH;
}

static SchedEnum running_handler(){

    command_status_t sts = command_get_status(m_scheduler.running_cmd);
    if (sts == DONE_OK || sts == ERR || sts == TIMEOUT){
        return ST_FINISH;
    }
    return ST_IN_PROGRESS;
}

static SchedEnum finish_handler(){

    if (m_scheduler.max_idx != 0){
        command_release(&m_scheduler.running_cmd);
    }
    if (command_get_count() == 0){
        vTaskSuspend(m_scheduler.taskHandle);
    }
    return ST_INIT;
}

static void update_cmd_status(command_status_t status){
    command_set_status(m_scheduler.running_cmd, status);
}

void Commands_Scheduler(){

    m_scheduler.next_state = m_scheduler.state_handler[m_scheduler.state]();

    m_scheduler.state = m_scheduler.next_state;
}

void Commands_Scheduler_Init(TaskHandle_t handle){

    command_buff_init();

    m_scheduler.taskHandle = handle;
}

void Commands_Scheduler_Resume(){
    // eTaskState state = eTaskGetState(m_scheduler.taskHandle);
    // if (state == eSuspended){
        xTaskResumeFromISR(m_scheduler.taskHandle);
    // }
}
