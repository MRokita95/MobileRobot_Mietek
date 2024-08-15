#ifndef COMM_H_
#define COMM_H_

#include <stdint.h>
#include <stdlib.h>
#include "FreeRTOS.h"

#define COMM_FRAME_PARAM_SIZE   10u
#define COMM_FRAME_BUFF_SIZE    10u     // place for max messages

#define COMMAND_FROM_CLIENT     0x01
#define PACKET_HEADER_UNREAD    0xAA
#define PACKET_HEADER_READ      0xBB
#define HK_DATA_HEADER          0x01u
#define TASK_FAILED_HEADER      0x02u
#define TRACE_DATA_HEADER       0x03u

typedef struct{
    uint8_t msg_id;
    uint8_t size;
}__attribute__((packed))header_t;

typedef struct{
    uint8_t application_id;
    uint8_t function_id;
    uint8_t parameters[COMM_FRAME_PARAM_SIZE];
}__attribute__((packed)) appdata_t;

typedef struct{
    header_t header;
    appdata_t appdata;
    uint8_t data_valid;
}__attribute__((packed)) comm_task_frame_t;

typedef struct{
    uint16_t task_error_code;
    uint16_t task_failed;
    uint16_t function_failed;
}__attribute__((packed)) comm_task_response_frame_t;


void Comm_Task(void);

void Comm_Init(void);

void Comm_Uart_Receive(void);

void Comm_RetreiveBuffer(comm_task_frame_t* comm_data, uint16_t* size);

#endif