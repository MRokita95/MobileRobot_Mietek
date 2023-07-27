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
 * @brief communication task frame buffer
 * 
 * @note size is the same as RX_BUFFER_SIZE
 * no need for bigger, as we read full buffer at once
 * 
 */
static comm_task_frame_t m_comm_buffer;
static SemaphoreHandle_t comm_buff_access;

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

    rx_write_pointer++;
    HAL_UART_Receive_IT(&huart2, rx_data, (uint16_t)MAX_UART_MSG_SIZE);
    

}



static void retreive_uart_data(){
    /*disable UART-based interrupts when we modify rx_write_pointer and read rx_buffer */
    HAL_NVIC_DisableIRQ(USART2_IRQn);

    int32_t packets_cnt = rx_write_pointer;

    if (packets_cnt <= 0){
        HAL_NVIC_EnableIRQ(USART2_IRQn);
        return;
    }

    uint16_t next_comm_buff_idx = 0u;

    
    

    //mutex take
    if( xSemaphoreTake(comm_buff_access, 5) == pdPASS){


        
        //TODO: check if application_id could stand here (one or two)
        memcpy(&m_comm_buffer.appdata, rx_data[MAX_UART_MSG_SIZE], sizeof(appdata_t));
        memcpy(&m_comm_buffer.data_valid, &rx_data[MAX_UART_MSG_SIZE-1], 1);

        m_comm_buffer.header = PACKET_HEADER_UNREAD;

        rx_write_pointer--;

        xQueueSend(xCommQueue, &m_comm_buffer, 5u);

        //mutex return
        xSemaphoreGive(comm_buff_access);


    }


    HAL_NVIC_EnableIRQ(USART2_IRQn);
}


void Comm_RetreiveBuffer(comm_task_frame_t* comm_data, uint16_t* size){

    if(comm_data == NULL || size == NULL){
        return;
    }

    *size = 0;
    if( xSemaphoreTake(comm_buff_access, 5) == pdPASS){
        
        comm_task_frame_t* packet = &m_comm_buffer;  //start always from the begining

        while(packet->header == PACKET_HEADER_UNREAD){
            memcpy(&comm_data[*size], packet, sizeof(comm_task_frame_t));
            packet->header = PACKET_HEADER_READ;

            packet++;
            *size++;
        }

        xSemaphoreGive(comm_buff_access);
    }
}


void Comm_Init(){
    rx_write_pointer = -1;
    
	comm_buff_access = xSemaphoreCreateMutex();

	configASSERT(comm_buff_access);

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

    retreive_uart_data();
}

