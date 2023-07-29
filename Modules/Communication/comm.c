#include "comm.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "commands.h"
#include "string.h"


#define MAX_UART_MSG_SIZE   11u      // single message max size + 1 byte of HW control stop
#define RX_BUFFER_SIZE      2u     // place for max messages


extern UART_HandleTypeDef huart2;
extern void send_uart(UART_HandleTypeDef* uart_instance, void const * argument);


/* robot housekeeping queue */
extern QueueHandle_t xHKQueue;

/**
 * @brief Commands Queue
 * 
 */
#define COMM_QUEUE_LENGTH   5u
#define COMM_FRAME_SIZE sizeof(comm_task_frame_t)
static StaticQueue_t xStaticQueue;
uint8_t queue_buffer[COMM_QUEUE_LENGTH * COMM_FRAME_SIZE];
QueueHandle_t xCommQueue;


/**
 * @brief Response Queue
 * 
 */
#define RESP_QUEUE_LENGTH   5u
#define RESP_FRAME_SIZE sizeof(comm_task_response_frame_t)
static StaticQueue_t xRespStaticQueue;
uint8_t resp_queue_buffer[COMM_QUEUE_LENGTH * COMM_FRAME_SIZE];
QueueHandle_t xRespQueue;




/**
 * @brief Receiving buffer and data pointer
 * 
 */
static uint8_t rx_buffer[MAX_UART_MSG_SIZE*RX_BUFFER_SIZE];
static int16_t rx_write_pointer = -1;   /* after initialization is set to valid value*/
static uint8_t rx_data[MAX_UART_MSG_SIZE];	

/**
 * @brief Receive bytes from uart and place into rx_buffer
 * 
 * @note Function called from interrupt (HAL_UART_RxCpltCallback)
 * 
 */
void Comm_Uart_Receive_Irq(){

    comm_task_frame_t m_comm_buffer;
    BaseType_t xHigherPriorityWoken = pdFALSE;

    memcpy(&m_comm_buffer.appdata, rx_data, sizeof(appdata_t));

    /* only verified data could be queued */
    if ((m_comm_buffer.appdata.application_id != 0u) && (m_comm_buffer.appdata.function_id != 0u)){
        memcpy(&m_comm_buffer.data_valid, &rx_data[MAX_UART_MSG_SIZE-1], 1);
        m_comm_buffer.header = PACKET_HEADER_UNREAD;

        xQueueSendFromISR(xCommQueue, &m_comm_buffer, &xHigherPriorityWoken);
    }
    
    /* set next interrupt service routing*/
    HAL_UART_Receive_IT(&huart2, rx_data, (uint16_t)MAX_UART_MSG_SIZE);
}


void Comm_Init(){

    xCommQueue = xQueueCreateStatic(COMM_QUEUE_LENGTH, COMM_FRAME_SIZE, queue_buffer, &xStaticQueue);
    configASSERT(xCommQueue);
    

    xRespQueue = xQueueCreateStatic(RESP_QUEUE_LENGTH, RESP_FRAME_SIZE, resp_queue_buffer, &xRespStaticQueue);
    configASSERT(xRespQueue);

	Comm_Uart_Receive_Irq();
}

void Comm_Task(){

    
    char *message;


    /* HouseKeeping data to send */
    if(xQueueReceive(xHKQueue, &message, 10u) == pdTRUE) {

        send_uart(&huart2, message);
    }


    comm_task_response_frame_t task_response;
    /* Response error data to send */
    if(xQueueReceive(xRespQueue, &task_response, 10u) == pdTRUE) {

        //temp solution
        uint16_t header = TASK_FAILED_HEADER;
        send_uart(&huart2, &header);
        send_uart(&huart2, &task_response.task_error_code);
        send_uart(&huart2, &task_response.task_failed);
        send_uart(&huart2, &task_response.function_failed);
    } 
}

