#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "robot.h"
#include "commands.h"
#include "comm_types.h"
#include "comm.h"
#include "param_handle.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "tracing.h"



typedef enum{
    STS_OK = 0,
    INVALID_APP_ID,
    INVALID_FUNC_ID,
    INVALID_PARAM,
    FAILED_EXEC
} task_exec_status_t;

/**
 * @brief Response for the task - only when error occurs
 * 
 */
static comm_task_response_frame_t m_task_response;

/**
 * @brief Trace Data buffer 
 * allocates on heap based on the required size
 * freed when last data element is sent
 * 
 */
trace_data_t* m_trace_data;
uint16_t m_tr_data_size;
bool m_transfer_ongoing;

/**
 * @brief Comm Frame for tasks
 * 
 */
extern QueueHandle_t xCommQueue;


/**
 * @brief Response to 
 * 
 */
extern QueueHandle_t xRespQueue;



/**
 * @brief Response to 
 * 
 */
extern QueueHandle_t xTraceQueue;
extern QueueHandle_t xRobDataQueue;

extern Mobile_Platform_t* robot;


static task_exec_status_t collect_robot_data(robot_status_data_t* rob_data){
    if (rob_data == NULL){
        return INVALID_PARAM;
    }

    rob_data->current_state = Robot_Status(robot);
    rob_data->active_mode = Robot_ActiveMode(robot);
    rob_data->speed_setpoint = robot->speed_setpoint;
    rob_data->right_wheel_speed = Robot_GetWheelSpeed(robot, RIGHT);
    rob_data->left_wheel_speed = Robot_GetWheelSpeed(robot, LEFT);

    return STS_OK;
}

static void send_robot_data(robot_status_data_t* rob_data){
    xQueueSend(xRobDataQueue, rob_data, portMAX_DELAY);
}


static inline bool trace_data_transfer_on(){
    return m_transfer_ongoing;
}


/**
 * @brief Send trace data to related queue
 * 
 * 
 */
static void send_trace_data(){
    
    static uint16_t data_size = 0;
    static trace_data_t* trace_ptr = NULL;

    if ((m_tr_data_size != 0) && !m_transfer_ongoing){
        m_transfer_ongoing = true;
        data_size = m_tr_data_size;
        trace_ptr = m_trace_data;
    } else { 
        xQueueSend(xTraceQueue, trace_ptr, portMAX_DELAY);
        trace_ptr++;
        data_size--;
        if (data_size == 0u){
            free(m_trace_data);
            m_tr_data_size = 0u;
            m_transfer_ongoing = false;
        }
    }
}



static bool deserialize_rob_command(command_type_t cmd_type, command_t* cmd, uint8_t* params){
    
    bool cmd_ok = false;

    *cmd = (command_t){0};

    size_t par_size;

    switch (cmd_type)
    {
    case START_ROB:
    case STOP_ROB:
    case AUTOPATH_START:
    case AUTOPATH_STOP:
    case MANUAL_START:
    case MANUAL_STOP:
        cmd->type = cmd_type;
        cmd_ok = true;
        break;
    
    case RUN_FOR_TIME:
        cmd->type = cmd_type;
        par_size = sizeof(cmd->speed);
        memcpy(&cmd->speed, params, par_size);

        params += par_size;
        par_size = sizeof(cmd->time);
        memcpy(&cmd->time, params, par_size);
        cmd_ok = true;
        break;

    case RUN_FOR_DIST:
        cmd->type = cmd_type;
        par_size = sizeof(cmd->speed);
        memcpy(&cmd->speed, params, par_size);

        params += par_size;
        par_size = sizeof(cmd->distance);
        memcpy(&cmd->distance, params, par_size);
        cmd_ok = true;
        break;

    case RUN_TO_POINT:
        cmd->type = cmd_type;
        par_size = sizeof(cmd->speed);
        memcpy(&cmd->speed, params, par_size);

        params += par_size;
        par_size = sizeof(cmd->point.x_pos);
        memcpy(&cmd->point.x_pos, params, par_size);

        params += par_size;
        par_size = sizeof(cmd->point.y_pos);
        memcpy(&cmd->point.y_pos, params, par_size);
        cmd_ok = true;
        break;

    case ROTATE:
        cmd->type = cmd_type;
        par_size = sizeof(cmd->speed);
        memcpy(&cmd->speed, params, par_size);

        params += par_size;
        par_size = sizeof(cmd->angle);
        memcpy(&cmd->angle, params, par_size);
        cmd_ok = true;
        break;

    case WAIT_TIME:
        cmd->type = cmd_type;
        par_size = sizeof(cmd->time);
        memcpy(&cmd->time, params, par_size);
        cmd_ok = true;
        break;
    
    default:
        break;
    }

    return cmd_ok;
}

static task_exec_status_t send_for_execution(comm_task_frame_t* task){

    task_exec_status_t status = STS_OK;

    switch (task->appdata.application_id)
    {
    case ROB_APP_ID:
    {
        command_type_t cmd_type = task->appdata.function_id;
        uint8_t* parameters = &task->appdata.parameters;  //TODO
        command_t rob_cmd;
        bool cmd_ok = deserialize_rob_command(cmd_type, &rob_cmd, parameters);
        if (cmd_ok){
            command_buff_status_t buff_status = command_add(rob_cmd);

            if (buff_status != BUFF_OK){
                status = FAILED_EXEC;
                m_task_response.task_error_code = (uint16_t)buff_status;
            }
        }
        break;
    }

    case MEM_APP_ID:
        switch (task->appdata.function_id)
        {
        case FLASH_SAVE_FNC_ID:
        {
            param_ret_status_t param_sts = Param_SaveToFlash();
            if (param_sts != PARAM_OK){
                status = FAILED_EXEC;
                m_task_response.task_error_code = (uint16_t)param_sts;
            }
            break;
        }
        case FLASH_LOAD_FNC_ID:
        {
            param_ret_status_t param_sts = Param_LoadFromFlash();
            if (param_sts != PARAM_OK){
                status = FAILED_EXEC;
                m_task_response.task_error_code = (uint16_t)param_sts;
            }
            break;
        }
        
        default:
            status = INVALID_FUNC_ID;
            break;
        }

    case PAR_APP_ID:
        switch (task->appdata.function_id)
        {
        case PARAM_SET_FNC_ID:
        {
            uint16_t par_id = 0u;
            memcpy(&par_id, &task->appdata.parameters[0], 2u);
            void* par_val; 
            memcpy(&par_val, &task->appdata.parameters[2], 4u);  //Get Max size
            param_ret_status_t param_sts = Param_Set(par_id, &par_val);
            if (param_sts != PARAM_OK){
                status = FAILED_EXEC;
                m_task_response.task_error_code = (uint16_t)param_sts;
            }
            break;
        }

        case PARAM_GET_FNC_ID:
        {
            uint16_t par_id = 0u;
            memcpy(&par_id, &task->appdata.parameters[0], 2u);
            uint16_t par_val = 0u;  //TODO:
            param_ret_status_t param_sts = Param_Get(par_id, &par_val);
            if (param_sts == PARAM_OK){
                status = STS_OK;
            } else {
                status = FAILED_EXEC;
                m_task_response.task_error_code = (uint16_t)param_sts;
            }
            break;
        }
        
        default:
            status = INVALID_FUNC_ID;
            break;
        }
    
    case HK_APP_ID:
    {
        /* task->appdata.function_id - not used  */
        robot_status_data_t rob_data;
        status = collect_robot_data(&rob_data);
        send_robot_data(&rob_data);
        break;
    }


    case TR_DATA_APP_ID:
    {
        /* task->appdata.function_id - not used  */
        uint16_t data_size = 0u;
        memcpy(&data_size, &task->appdata.parameters[0], 2u);

        if (data_size == 0u){
            status = INVALID_PARAM;
            break;
        } else if ((m_tr_data_size != 0u) || trace_data_transfer_on()){
            status = FAILED_EXEC;
            break;
        }

        /* Size is OK, lets retreive data from the buffer but first allocate the buffer */
        m_tr_data_size = data_size;
        m_trace_data = malloc(sizeof(trace_data_t) * m_tr_data_size);   /* maybe not all data would be needed, but the maximum must be allocated */
        Trace_FlushData(m_trace_data, &m_tr_data_size);

        send_trace_data();
        break;
    }



    
    default:
        status = INVALID_APP_ID;
        break;
    }

    return status;
}



void Management_Task(){

    comm_task_frame_t task_buff[COMM_FRAME_BUFF_SIZE];
    //uint16_t size=0u;
    //Comm_RetreiveBuffer(&task_buff[0], &size);

    //UBaseType_t cnt = uxQueueMessagesWaiting(xCommQueue);

    /* trace transfer is ongoing - continue */
    if (trace_data_transfer_on()){
        send_trace_data();
    }


    if(xQueueReceive(xCommQueue, &task_buff, 10u) == pdTRUE) {

        for (uint16_t idx = 0; idx < COMM_FRAME_BUFF_SIZE; idx++){

            if (task_buff[idx].header.msg_id == PACKET_HEADER_UNREAD){

                if (send_for_execution(&task_buff[idx]) != STS_OK){

                    m_task_response.task_failed = task_buff[idx].appdata.application_id;
                    m_task_response.function_failed = task_buff[idx].appdata.function_id;

                    xQueueSend(xRespQueue, &m_task_response, 10);
                    break;
                }
                task_buff[idx].header.msg_id = PACKET_HEADER_READ;
            }
        }

        // if (size > 0u){
        //     for (uint16_t idx = 0; idx < size; idx++){
        //         send_for_execution(&task_buff[idx]);
        //     }
        // }
    }

    
}
