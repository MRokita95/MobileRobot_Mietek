#include "tracing.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <string.h>

#define TRACE_BUFFER_SIZE 200u

#define COORD_INCREEMENT    10
#define ORIENT_INCREEMENT   5.f

typedef struct{
    uint16_t read_pointer;
    uint16_t write_pointer;
    uint32_t last_address;
    uint32_t next_write_address;
    uint8_t data_error;
    trace_data_t last_data;
    SemaphoreHandle_t buffer_access;
}trace_data_handling_t;

trace_data_handling_t m_trace_handler;

static trace_data_t m_trace_data_buffer[TRACE_BUFFER_SIZE];


static const Mobile_Platform_t* robot_instance;


static bool pos_changed(trace_data_t new_data, trace_data_t last_data){

    bool changed = false;
    if ((abs(new_data.rob_pos.x_pos - last_data.rob_pos.x_pos) >= COORD_INCREEMENT) ||
        (abs(new_data.rob_pos.y_pos - last_data.rob_pos.y_pos) >= COORD_INCREEMENT)){
        changed = true;
    }

    if ((fabs(new_data.rob_orient.pitch - last_data.rob_orient.pitch) >= ORIENT_INCREEMENT) ||
        (fabs(new_data.rob_orient.roll - last_data.rob_orient.roll) >= ORIENT_INCREEMENT)){
        changed = true;
    }

    return changed;
}


void Trace_InitAccessInstances(Mobile_Platform_t *robot){
    robot_instance = robot;

    m_trace_handler.buffer_access = xSemaphoreCreateMutex();
}

static inline void clamp_pointer(uint16_t* pointer){
    if (*pointer >= TRACE_BUFFER_SIZE){
        *pointer = 0u;
    }
}

void Trace_PullData(){

    /* Get robot data */
    trace_data_t new_data;
    new_data.timestamp = HAL_GetTick();
    new_data.rob_pos = Robot_GetCoord(robot_instance);
    new_data.rob_orient = Robot_GetCoord(robot_instance);

    if (!pos_changed(new_data, m_trace_handler.last_data)){
        /* Data not changed enaught to insert into buffer */
        return; 
    }

    if( xSemaphoreTake(m_trace_handler.buffer_access, 10) == pdPASS){
        uint32_t *buffer = m_trace_handler.next_write_address;

        memcpy(buffer, &new_data, sizeof(trace_data_t));
        memcpy(&m_trace_handler.last_data, &new_data, sizeof(trace_data_t));

        xSemaphoreGive(m_trace_handler.buffer_access);
    }


    /* handle with the write_pointer */
    uint16_t current_write_pointer = m_trace_handler.write_pointer;

    current_write_pointer++;
    clamp_pointer(&current_write_pointer);

    

    /* buffer full condition check */
    if (current_write_pointer == m_trace_handler.read_pointer) {

        /*
        * buffer is full, must be emptied from the oldest data in order (those data must remain in the buffer),
        * so the last tracind data saving will unforntunately be lost, if data will not be pulled
        * from the buffer before.
        * next_write_address will not be updated
        */
       
    } else {
        
        m_trace_handler.write_pointer++;
        clamp_pointer(&m_trace_handler.write_pointer);

        /*
         * next_write_address could be updated with the next free memory address in the buffer.
         * write_pointer will
        */
        m_trace_handler.next_write_address = &m_trace_data_buffer[m_trace_handler.write_pointer];
    }

}