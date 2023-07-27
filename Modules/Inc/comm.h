#ifndef COMM_H_
#define COMM_H_

#include <stdint.h>
#include <stdlib.h>

#define COMM_FRAME_PARAM_SIZE   10u
#define COMM_FRAME_BUFF_SIZE    10u     // place for max messages

#define PACKET_HEADER_UNREAD    0xAA00
#define PACKET_HEADER_READ      0xBB99
#define TASK_FAILED_HEADER      0xABABu

typedef struct{
    uint8_t application_id;
    uint8_t function_id;
    uint8_t parameters[COMM_FRAME_PARAM_SIZE];
} appdata_t;

typedef struct{
    uint16_t header;
    appdata_t appdata;
    uint8_t data_valid;
} comm_task_frame_t;

typedef struct{
    uint16_t task_error_code;
    uint16_t task_failed;
    uint16_t function_failed;
} comm_task_response_frame_t;


void Comm_Task(void);

void Comm_Init(void);

void Comm_Uart_Receive(void);

void Comm_RetreiveBuffer(comm_task_frame_t* comm_data, uint16_t* size);

#endif