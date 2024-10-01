#include "uart_dma.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "cmsis_os.h"

typedef struct
{
  __IO uint32_t ISR;   /*!< DMA interrupt status register */
  __IO uint32_t Reserved0;
  __IO uint32_t IFCR;  /*!< DMA interrupt flag clear register */
} DMA_Base_Registers;



void UARTDMA_Init(UARTDMA_HandleTypeDef *huartdma, UART_HandleTypeDef *huart){
    huartdma->huart = huart;

    huartdma->readPtr = 0u;
    huartdma->writePtr = 0u;
    huartdma->msgCnt = 0u;

    __HAL_UART_ENABLE_IT(huartdma->huart, UART_IT_IDLE);
    __HAL_DMA_ENABLE_IT(huartdma->huart->hdmarx, DMA_IT_TC);

    HAL_UART_Receive_DMA(huartdma->huart, huartdma->dmaRxBuffer, DMA_RX_BUFFER_SIZE); // Run DMA for whole DMA buffer

	huartdma->huart->hdmarx->Instance->CR &= ~DMA_SxCR_HTIE; // Disable DMA Half Complete interrupt
}

void UARTDMA_UartIrqHandler(UARTDMA_HandleTypeDef *huartdma){

    if(huartdma->huart->Instance->SR & UART_FLAG_IDLE)       // Check if Idle flag is set
	{
		volatile uint32_t tmp;
		tmp = huartdma->huart->Instance->SR;                      // Read status register
		tmp = huartdma->huart->Instance->DR;                      // Read data register
		huartdma->huart->hdmarx->Instance->CR &= ~DMA_SxCR_EN; // Disable DMA - it will force Transfer Complete interrupt if it's enabled
	}
}

void UARTDMA_DmaIrqHandler(UARTDMA_HandleTypeDef *huartdma){

    DMA_Base_Registers *regs = (DMA_Base_Registers *)huartdma->huart->hdmarx->StreamBaseAddress;

    if(__HAL_DMA_GET_IT_SOURCE(huartdma->huart->hdmarx, DMA_IT_TC) != RESET){

        regs->IFCR = DMA_FLAG_TCIF0_4 << huartdma->huart->hdmarx->StreamIndex;	// Clear Transfer Complete flag

        //circullar buffer handling
        const uint32_t length = DMA_RX_BUFFER_SIZE - huartdma->huart->hdmarx->Instance->NDTR;
        
        const uint16_t startWritePtr = huartdma->writePtr;
        for (uint16_t idx = 0; idx <length; idx++){

            huartdma->uartCircBuffer[idx + startWritePtr] = huartdma->dmaRxBuffer[idx];
            huartdma->writePtr++;
            if (huartdma->writePtr >= UART_CIRC_BUFFER_SIZE){
                huartdma->writePtr = 0;
            }
        }
        huartdma->msgCnt++;

    }

    regs->IFCR = 0x3FU << huartdma->huart->hdmarx->StreamIndex; 		// Clear all interrupts
	huartdma->huart->hdmarx->Instance->M0AR = (uint32_t) huartdma->dmaRxBuffer; // Set memory address for DMA again
	huartdma->huart->hdmarx->Instance->NDTR = DMA_RX_BUFFER_SIZE; // Set number of bytes to receive
	huartdma->huart->hdmarx->Instance->CR |= DMA_SxCR_EN;            	// Start DMA transfer

    // DmaRegisters->IFCR = 0x3FU << huartdma->huart->hdmarx->StreamIndex;
    // //__HAL_UART_ENABLE_IT(huartdma->huart, UART_IT_IDLE);
    // __HAL_DMA_ENABLE_IT(huartdma->huart->hdmarx, DMA_IT_TC);
    // HAL_UART_Receive_DMA(huartdma->huart, huartdma->dmaRxBuffer, DMA_RX_BUFFER_SIZE); // Run DMA for whole DMA buffer
}

uint16_t UARTDMA_DataCount(UARTDMA_HandleTypeDef *huartdma){
    return huartdma->msgCnt;
}

void UARTDMA_GetData(UARTDMA_HandleTypeDef *huartdma, uint8_t* data, uint16_t read_bytes){

    const uint16_t startReadPtr = huartdma->readPtr;
    const uint16_t length = read_bytes;

    for (uint16_t idx = 0; idx <length; idx++){

        data[idx] = huartdma->uartCircBuffer[idx+startReadPtr];
        huartdma->readPtr++;
        if (huartdma->readPtr >= UART_CIRC_BUFFER_SIZE){
            huartdma->readPtr = 0;
        }
    }
    huartdma->msgCnt--;
}
