#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "robot.h"
#include "mobile_platform.h"
#include "FreeRTOS.h"
#include "queue.h"


//queue definition
#define QUEUE_LENGTH   20
#define MESSAGE_LENGTH 200
#define MESSAGE_PTR sizeof(char*)
static StaticQueue_t xStaticQueue;
uint8_t queue_buffer[QUEUE_LENGTH * MESSAGE_PTR];
QueueHandle_t xHKQueue;

//single housekeeping message buffer
char hk_message_buff[MESSAGE_LENGTH];

//robot coordinates
static rob_coord_t robot_actual_pos;
static robot_status_t robot_status = ROB_OFF;

static uint32_t last_update = 0u;

static char* robot_status_to_msg(robot_status_t status){
    char* status_msg;
    switch (status)
    {
    case ROB_OFF:
        status_msg = "OFF";
        break;
    case ROB_IN_PROGRESS:
        status_msg = "WORKING";
        break;
    case ROB_IDLE:
        status_msg = "IDLE";
    default:
        break;
    }
    return status_msg;
}

void HK_Init(Mobile_Platform_t* robot){

    xHKQueue = xQueueCreateStatic(QUEUE_LENGTH, MESSAGE_PTR, queue_buffer, &xStaticQueue);
    configASSERT(xHKQueue);
}

void HK_Update(Mobile_Platform_t* robot){

    if ((last_update - HAL_GetTick()) < HK_UPDATE){
        return;
    }

    char* message = &hk_message_buff[0];

    rob_coord_t new_pos = Robot_GetCoord(robot);
    robot_status_t new_status = Robot_Status(robot);
    float new_orient = Robot_GetOrient(robot);

    uint32_t n_bytes = 0;
    if (new_pos.x_pos != robot_actual_pos.x_pos || new_pos.y_pos != robot_actual_pos.y_pos || new_status != robot_status || robot->actual_speed != 0)
    {
        n_bytes +=  sprintf(&message[n_bytes], "ROB POS: %i [x], %i [y], %i [z], ORIENT: %f ACT SPEED: %i, ACT DIST: %u\r\n",  
                                                                                new_pos.x_pos, 
                                                                                new_pos.y_pos, 
                                                                                new_pos.z_pos,
                                                                                new_orient,
                                                                                robot->actual_speed,
                                                                                robot->current_distance);

        n_bytes += sprintf(&message[n_bytes], "STATUS: %s", robot_status_to_msg(new_status));

        xQueueSend(xHKQueue, &message, portMAX_DELAY);
    }

    robot_actual_pos = new_pos;
    robot_status = new_status;
    
    last_update = HAL_GetTick();
}

void HK_Setpoints(void* setpoint, current_setpoint_t setpoint_type){

    char* message = &hk_message_buff[0];

    switch (setpoint_type)
    {
    case TIMER:
    	{
    		uint32_t setp = *(uint32_t*)setpoint;
    		sprintf(message, "TIMER ON, setpoint: %u\r\n", setp);
    	}
        break;

    case DISTANCE:
    	{
    		int32_t setp = *(int32_t*)setpoint;
    		sprintf(message, "DISTANCE ON, setpoint: %i\r\n", setp);
    	}
    	break;

    case POINT:
    	{
    		rob_coord_t setp = *(rob_coord_t*)setpoint;
    		sprintf(message, "MOVE TO POINT ON, setpoint: %i [x] %i [y] \r\n", setp.x_pos, setp.y_pos);
    	}
        break;
    
    default:
        break;
    }

    xQueueSend(xHKQueue, &message, portMAX_DELAY);
}
