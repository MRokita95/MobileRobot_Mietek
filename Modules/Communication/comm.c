#include "comm.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "queue.h"
#include "commands.h"
#include "string.h"
#include "tracing.h"

#include "uart_dma.h"

#define START_FRAME_SIGN    "{"
#define END_FRAME_SIGN      "}"

#define MAX_UART_MSG_SIZE   14u      // single message max size
#define RX_BUFFER_SIZE      2u     // place for max messages


extern UART_HandleTypeDef huart2;
extern UARTDMA_HandleTypeDef huartdma;
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
 * @brief Trace Queue - contains robot trace info data
 * Queue length = 1 could be enough because management task has 5 sec period, and comm Task has 500 ms
 * and we send only 1 trace data per management task.
 */
#define TRACE_QUEUE_LENGTH   4u
#define TRACE_FRAME_SIZE sizeof(trace_data_t)
static StaticQueue_t xTraceStaticQueue;
uint8_t trace_queue_buffer[TRACE_QUEUE_LENGTH * TRACE_FRAME_SIZE];
QueueHandle_t xTraceQueue;

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
void Comm_Uart_Receive_Poll(){

    comm_task_frame_t m_comm_buffer;
    BaseType_t xHigherPriorityWoken = pdFALSE;

    if (HAL_UART_Receive(&huart2, rx_data, (uint16_t)MAX_UART_MSG_SIZE, 500) == HAL_OK){

    	memcpy(&m_comm_buffer.header, rx_data, sizeof(header_t));

		if (m_comm_buffer.header.msg_id == COMMAND_FROM_CLIENT && m_comm_buffer.header.size > 0){
			memcpy(&m_comm_buffer.appdata, &rx_data[sizeof(header_t)], m_comm_buffer.header.size);

			/* only verified data could be queued */
			if ((m_comm_buffer.appdata.application_id != 0u) && (m_comm_buffer.appdata.function_id != 0u)){
				memcpy(&m_comm_buffer.data_valid, &rx_data[MAX_UART_MSG_SIZE-1], 1);
				m_comm_buffer.header.msg_id = PACKET_HEADER_UNREAD;

				xQueueSend(xCommQueue, &m_comm_buffer, &xHigherPriorityWoken);
			}
		}
    }
    
    /* set next interrupt service routing*/
    HAL_UART_Receive_IT(&huart2, rx_data, (uint16_t)MAX_UART_MSG_SIZE);
}

static void read_uart_data(){
    comm_task_frame_t m_comm_buffer;
    BaseType_t xHigherPriorityWoken = pdFALSE;

    while (UARTDMA_DataCount(&huartdma) > 0){

        portDISABLE_INTERRUPTS();

        UARTDMA_GetData(&huartdma, &m_comm_buffer.header, sizeof(header_t), false);
        //m_comm_buffer.header.size >= 2u -> apid + funcid
        if (m_comm_buffer.header.msg_id == COMMAND_FROM_CLIENT && m_comm_buffer.header.size >= 2u){
            UARTDMA_GetData(&huartdma, &m_comm_buffer.appdata, m_comm_buffer.header.size, true);

            /* only verified data could be queued */
            if ((m_comm_buffer.appdata.application_id != 0u) || (m_comm_buffer.appdata.function_id != 0u)){

                m_comm_buffer.data_valid = 1;
                m_comm_buffer.header.msg_id = PACKET_HEADER_UNREAD;
                xQueueSend(xCommQueue, &m_comm_buffer, &xHigherPriorityWoken);
            }
        }

        portENABLE_INTERRUPTS();
    }
}

void Comm_Init(){

    xCommQueue = xQueueCreateStatic(COMM_QUEUE_LENGTH, COMM_FRAME_SIZE, queue_buffer, &xStaticQueue);
    configASSERT(xCommQueue);
    

    xRespQueue = xQueueCreateStatic(RESP_QUEUE_LENGTH, RESP_FRAME_SIZE, resp_queue_buffer, &xRespStaticQueue);
    configASSERT(xRespQueue);

    xTraceQueue = xQueueCreateStatic(TRACE_QUEUE_LENGTH, TRACE_FRAME_SIZE, trace_queue_buffer, &xTraceStaticQueue);
    configASSERT(xTraceQueue);

    UARTDMA_Init(&huartdma, &huart2);
	//Comm_Uart_Receive_Irq();
}

void Comm_Task(){


    read_uart_data();
    //Comm_Uart_Receive_Poll();
    
    char *message;

    /* HouseKeeping data to send */
    if(xQueueReceive(xHKQueue, &message, 10u) == pdTRUE) {
        header_t header;
        header.msg_id = HK_DATA_HEADER;
        header.size = 0;    //using this ID, size is not relevant
        
        //send_uart(&huart2, &(uint8_t){START_FRAME_SIGN});
        send_uart(&huart2, &header);
        send_uart(&huart2, message);
        //send_uart(&huart2, &(uint8_t){END_FRAME_SIGN});
        return;     /* one message per Communication Frame */
    }


    comm_task_response_frame_t task_response;
    /* Response error data to send */
    if(xQueueReceive(xRespQueue, &task_response, 10u) == pdTRUE) {

        //temp solution
        header_t header;
        header.msg_id = TASK_FAILED_HEADER;
        header.size = sizeof(comm_task_response_frame_t);

        send_uart(&huart2, &(uint8_t){START_FRAME_SIGN});
        send_uart(&huart2, &header);
        send_uart(&huart2, &task_response.task_error_code);
        send_uart(&huart2, &task_response.task_failed);
        send_uart(&huart2, &task_response.function_failed);
        send_uart(&huart2, &(uint8_t){END_FRAME_SIGN});
        return;     /* one message per Communication Frame */
    } 


    static char tr_message_buff[sizeof(header_t)+sizeof(trace_data_t)+2];
    trace_data_t trace_data;
    /* Response error data to send */
    if(xQueueReceive(xTraceQueue, &trace_data, 10u) == pdTRUE) {

        //temp solution TODO: 
        header_t header;
        header.msg_id = TRACE_DATA_HEADER;
        header.size = TRACE_FRAME_SIZE;

        // trace_data.timestamp = 999;
        // trace_data.rob_pos.x_pos = 10;
        // trace_data.rob_pos.y_pos = 20;
        // trace_data.rob_pos.z_pos = 0;
        // trace_data.rob_orient.roll = 90;
        // trace_data.rob_orient.pitch = 0;
        // trace_data.rob_orient.yaw = 0;

        //send_uart(&huart2, &(uint8_t){START_FRAME_SIGN});
        memcpy(tr_message_buff, &trace_data, sizeof(trace_data_t));
        tr_message_buff[sizeof(trace_data_t)] = '\r';
        tr_message_buff[sizeof(trace_data_t)+1] = '\n';

        send_uart(&huart2, &header);
        //send_uart(&huart2, tr_message_buff);    //check return code???
        HAL_UART_Transmit(&huart2, (uint8_t *)tr_message_buff, sizeof(trace_data_t)+2, 100);
        //send_uart(&huart2, &(uint8_t){END_FRAME_SIGN});
        return;     /* one message per Communication Frame */
    }
}

