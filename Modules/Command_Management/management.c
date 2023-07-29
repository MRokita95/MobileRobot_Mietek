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



typedef enum{
    OK = 0,
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
 * @brief Comm Frame for tasks
 * 
 */
extern QueueHandle_t xCommQueue;


/**
 * @brief Response to 
 * 
 */
extern QueueHandle_t xRespQueue;



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

    task_exec_status_t status;

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

            if (buff_status == BUFF_OK){
                status = OK;
            } else {
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
            if (param_sts == PARAM_OK){
                status = OK;
            } else {
                status = FAILED_EXEC;
                m_task_response.task_error_code = (uint16_t)param_sts;
            }
            break;
        }
        case FLASH_LOAD_FNC_ID:
        {
            param_ret_status_t param_sts = Param_LoadFromFlash();
            if (param_sts == PARAM_OK){
                status = OK;
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
            if (param_sts == PARAM_OK){
                status = OK;
            } else {
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
                status = OK;
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
    
    default:
        break;
    }
}



void Management_Task(){

    comm_task_frame_t task_buff[COMM_FRAME_BUFF_SIZE];
    //uint16_t size=0u;
    //Comm_RetreiveBuffer(&task_buff[0], &size);

    //UBaseType_t cnt = uxQueueMessagesWaiting(xCommQueue);

     /* HouseKeeping data to send */
    if(xQueueReceive(xCommQueue, &task_buff, 10u) == pdTRUE) {

        for (uint16_t idx = 0; idx < COMM_FRAME_BUFF_SIZE; idx++){

            if (task_buff[idx].header == PACKET_HEADER_UNREAD){

                if (send_for_execution(&task_buff[idx]) != OK){

                    m_task_response.task_failed = task_buff[idx].appdata.application_id;
                    m_task_response.function_failed = task_buff[idx].appdata.function_id;

                    xQueueSend(xRespQueue, &m_task_response, 10);
                    break;
                }

            }
        }

        // if (size > 0u){
        //     for (uint16_t idx = 0; idx < size; idx++){
        //         send_for_execution(&task_buff[idx]);
        //     }
        // }
    }

    
}
