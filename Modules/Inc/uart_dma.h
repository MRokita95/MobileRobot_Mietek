#ifndef UART_DMA_H_
#define UART_DMA_H_

#include "stdlib.h"
#include "stdbool.h"
#include "stdint.h"
#include "stm32f4xx_hal.h"


#define DMA_RX_BUFFER_SIZE (20u)
#define UART_CIRC_BUFFER_SIZE (1024u)


typedef struct
{
  UART_HandleTypeDef* huart;
  uint8_t dmaRxBuffer[DMA_RX_BUFFER_SIZE];
  uint8_t uartCircBuffer[UART_CIRC_BUFFER_SIZE];
  uint16_t writePtr;
  uint16_t readPtr;
  uint8_t msgCnt;
}UARTDMA_HandleTypeDef;

void UARTDMA_Init(UARTDMA_HandleTypeDef *huartdma, UART_HandleTypeDef *huart);

void UARTDMA_UartIrqHandler(UARTDMA_HandleTypeDef *huartdma);
void UARTDMA_DmaIrqHandler(UARTDMA_HandleTypeDef *huartdma);

uint16_t UARTDMA_DataCount(UARTDMA_HandleTypeDef *huartdma);
void UARTDMA_GetData(UARTDMA_HandleTypeDef *huartdma, uint8_t* data, uint16_t read_bytes, bool msg_read);

#endif
