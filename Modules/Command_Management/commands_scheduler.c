#include "commands.h"
#include "commands_ops.h"
#include "application_defs.h"
#include "task.h"
#include "timers.h"

#define NOTIF_VALUE 1u

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
    TimerHandle_t timerHandle;
}SchedState_t;

static SchedEnum init_handler(void);
static SchedEnum select_handler(void);
static SchedEnum dispatch_handler(void);
static SchedEnum running_handler(void);
static SchedEnum finish_handler(void);
static void command_status_notif(command_status_t status, int32_t retval);
static void timeoutCallback(TimerHandle_t timer);
static void sendNotification();

static SchedState_t m_scheduler = {
    .state = ST_INIT,
    .state_handler[ST_INIT] = init_handler,
    .state_handler[ST_SELECTION] = select_handler,
    .state_handler[ST_DISPATCH] = dispatch_handler,
    .state_handler[ST_IN_PROGRESS] = running_handler,
    .state_handler[ST_FINISH] = finish_handler,
    .running_cmd = NULL,
};

static command_severity_t check_queues(uint16_t* count){

    *count = 0;
    for (command_severity_t severity = CRITICAL_SEVERITY; (severity >= NORMAL_SEVERITY && severity <= CRITICAL_SEVERITY); severity--){
        *count = command_get_count(severity);
    	if (*count > 0){
            return severity; 
        }
    }
    return MAX_SEVERITY;
}

static SchedEnum init_handler(){
    command_severity_t severity = check_queues(&m_scheduler.max_idx);
    if (m_scheduler.max_idx == 0){
        return ST_FINISH;
    }
    command_buff_status_t buff_status = command_get_next(&m_scheduler.running_cmd, severity);
    if (buff_status == BUFF_EMPTY || command_get_status(m_scheduler.running_cmd) == EMPTY){
        command_release(m_scheduler.running_cmd);
        return ST_FINISH;
    }
    command_status_notif(QUEUED, 0);
    return ST_SELECTION;
}

static SchedEnum select_handler(){

    if (command_get_status(m_scheduler.running_cmd) == QUEUED){
        if (m_scheduler.running_cmd->guard(&m_scheduler.running_cmd->payload)){
            command_status_notif(READY, 0);
            if (m_scheduler.running_cmd->timeout == 0){
                m_scheduler.running_cmd->timeout = 125; //because
            }
            //xTimerChangePeriod(m_scheduler.timerHandle, m_scheduler.running_cmd->timeout, 0);
            return ST_DISPATCH;
        }
    }
    return ST_SELECTION;
}

static SchedEnum dispatch_handler(){

    if (command_get_status(m_scheduler.running_cmd) == READY){
        m_scheduler.running_cmd->dispatcher(&m_scheduler.running_cmd->payload, command_status_notif);
        //xTimerStart(m_scheduler.timerHandle, 0);
        return ST_IN_PROGRESS;
    }
    return ST_FINISH;
}

static SchedEnum running_handler(){

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(m_scheduler.running_cmd->timeout)) == NOTIF_VALUE){
        uint16_t counts;
        if (check_queues(&counts) == CRITICAL_SEVERITY){
            command_status_notif(INTERRUPTED, 0);
        }
    } else {
        command_status_notif(TIMEOUT, 0);   //instead of timer?
    }
    //xTimerStop(m_scheduler.timerHandle, 0);
    return ST_FINISH;
}

static SchedEnum finish_handler(){

    //Should I do sth with retval???
    if (m_scheduler.max_idx != 0){
        command_release(m_scheduler.running_cmd);
    }
    uint16_t count;
    if (check_queues(&count) == MAX_SEVERITY){
        vTaskSuspend(m_scheduler.taskHandle);
    }
    return ST_INIT;
}

static void command_status_notif(command_status_t status, int32_t retval){
    command_set_status(m_scheduler.running_cmd, status);
    if (status == DONE_OK || status == ERR){
        m_scheduler.running_cmd->retval = retval;
        sendNotification();
    }
}

static void timeoutCallback(TimerHandle_t timer){
    command_status_notif(TIMEOUT, 0);

}

static void sendNotification(){
    xTaskNotify(m_scheduler.taskHandle, NOTIF_VALUE, eSetValueWithOverwrite);
}

void Commands_Scheduler(){

    m_scheduler.next_state = m_scheduler.state_handler[m_scheduler.state]();

    m_scheduler.state = m_scheduler.next_state;
}

void Commands_Scheduler_Init(TaskHandle_t handle){

    command_buff_init();

    command_add_incoming_notif(CRITICAL_SEVERITY, sendNotification);

    m_scheduler.taskHandle = handle;
    //m_scheduler.timerHandle = xTimerCreate("CommandTimeout", 100, pdFALSE, ( void * ) 0, timeoutCallback);
}

void Commands_Scheduler_Resume(){
    // eTaskState state = eTaskGetState(m_scheduler.taskHandle);
    // if (state == eSuspended){
        xTaskResumeFromISR(m_scheduler.taskHandle);
    // }
}
