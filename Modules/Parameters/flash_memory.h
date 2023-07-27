#ifndef FLASH_MEMORY_H_
#define FLASH_MEMORY_H_


#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>



#define FLASH_START_PARAM_ADDR 0x08040000u

uint32_t Flash_Write(uint32_t StartAddress, uint32_t *txData, uint16_t size);

void Flash_Read(uint32_t StartAddress, uint32_t *rxData, uint16_t size);

#endif
