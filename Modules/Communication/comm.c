#include "comm.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "commands.h"

extern UART_HandleTypeDef huart2;
extern void send_uart(UART_HandleTypeDef* uart_instance, void const * argument);

static uint8_t rx_data[20];

/* robot housekeeping queue */
extern QueueHandle_t xHKQueue;

void Comm_Task(){

    
    char *message;

    /* HouseKeeping data to send */
    if(xQueueReceive(xHKQueue, &message, 10u) == pdTRUE) {

        send_uart(&huart2, message);
    }

    //TODO: receive commands via UART (buetooth)
}

