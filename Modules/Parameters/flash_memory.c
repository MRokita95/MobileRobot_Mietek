//#include "stm32f4xx_hal_flash_ex.h"
//#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal.h"
#include "flash_memory.h"

static uint32_t GetSector(uint32_t address){

    if (address >= FLASH_BASE && address <= 0x08003FFF){
        return FLASH_SECTOR_0;
    } 
    else if (address >= 0x08004000 && address <= 0x08007FFF){
        return FLASH_SECTOR_1;
    } 
    else if (address >= 0x08008000 && address <= 0x0800BFFF){
        return FLASH_SECTOR_2;
    }
    else if (address >= 0x0800C000 && address <= 0x0800FFFF){
        return FLASH_SECTOR_3;
    }
    else if (address >= 0x08010000 && address <= 0x0801FFFF){
        return FLASH_SECTOR_4;
    }
    else if (address >= 0x08020000 && address <= 0x0803FFFF){
        return FLASH_SECTOR_5;
    }
    else if (address >= 0x08040000 && address <= 0x0805FFFF){
        return FLASH_SECTOR_6;
    }
    else if (address >= 0x08060000 && address <= FLASH_END){
        return FLASH_SECTOR_7;
    }
}

uint32_t Flash_Write(uint32_t StartAddress, uint32_t *txData, uint16_t size){

    static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SectorError;
	uint16_t left=0;

    uint32_t program_address = StartAddress;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    /* Erase the user Flash area*/
    uint32_t StartSector = GetSector(StartAddress);
	uint32_t EndSectorAddress = StartAddress + size;
	uint32_t EndSector = GetSector(EndSectorAddress);

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    //EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.Sector = StartSector;
    EraseInitStruct.NbSectors = (EndSector - StartSector) + 1u;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;


    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGAERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGPERR);
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGSERR);


	if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		/*Error occurred while page erase.*/
		HAL_FLASH_Lock();
		return HAL_FLASH_GetError ();
	}


    /* Program the user Flash area word by word*/

    while (left<size)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, program_address, txData[left]) == HAL_OK)
        {
            program_address += 4u;  // use StartAddress += 2 for half word and 8 for double word
            left++;
        }
        else
        {
        /* Error occurred while writing data in Flash memory*/
        	HAL_FLASH_Lock();
            return HAL_FLASH_GetError ();
        }
    }

    /* Lock the Flash to disable the flash control register access (recommended
        to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();

    return 0;
}


void Flash_Read(uint32_t StartAddress, uint32_t *rxData, uint16_t size)
{
	while (1)
	{
		*rxData = *(volatile uint32_t *)StartAddress;
		StartAddress += 4u;
		rxData++;
		if (!(size--)) break;
	}
}

