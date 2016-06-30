<<<<<<< HEAD
/*
 * internal_flash_api.h
 *
 *  Created on: 11. Apr. 2016
 *      Author: martin.tomasini2
 */
#include "internal_flash_api.h"
#include "cmsis.h"

int WriteFlash(uint32_t ui32Address, const uint8_t* pData, uint32_t ui32DataSize)
{
    volatile uint16_t uintDataBuffer;

    HAL_FLASH_Unlock();

    // Write payload byte per byte
    for(uint32_t i = 0; i < ui32DataSize; i += 2, ui32Address += 2)
    {
        if(i < ui32DataSize - 1)
        {
            uintDataBuffer = pData[i];
            uintDataBuffer +=  (uint16_t)(pData[i + 1] << 8);
        }
        else
        {
            uintDataBuffer = pData[i];
            uintDataBuffer += 0xFF00;
        }
        if(HAL_OK !=  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, ui32Address, (uint64_t)uintDataBuffer))
        {
            return -1;
        }
    }

    HAL_FLASH_Lock();

    return 0;

}

int ReadFlash(uint32_t pui32Address, uint8_t* pData, uint32_t ui32DataSize)
{

}

int EraseSector(uint32_t ui32Sector)
{

}

int GetNumOfSectors(uint32_t* pui32NumOfSectors)
{

}

int GetSectorStartAdd(uint32_t ui32Sector, uint32_t* pui32Address)
{

}

int GetSectorEndAdd(uint32_t ui32Sector, uint32_t* pui32Address)
{

}


=======
/*
 * internal_flash_api.h
 *
 *  Created on: 11. Apr. 2016
 *      Author: martin.tomasini2
 */
#include "internal_flash_api.h"
#include "cmsis.h"


#ifdef TARGET_LIKE_STM32F091RC
    #define NUMBER_OF_FLASH_SECTORS 64
    #define SECTOR_SIZE 4096
    #define FLASH_SECTOR_0_START_ADDRESS  0x8000000
    #define FLASH_LAST_SECTOR_END_ADDRESS 0x0803FFFF
#else
    #error "MISSING OR WRONG MCU TARGET OR FOR FLASH DRIVER DEFINED"
#endif


int WriteFlash(uint32_t ui32Address, const uint8_t* pData, uint32_t ui32DataSize)
{
    if((ui32Address > FLASH_LAST_SECTOR_END_ADDRESS) || (ui32Address < FLASH_SECTOR_0_START_ADDRESS))
    {
        return -2;
    }

    volatile uint16_t uintDataBuffer;

    // Check if it is 2 bytes aligned
    if(0 != (ui32Address & 1))
    {
        return -3;
    }

    HAL_FLASH_Unlock();

    // Write payload byte per byte
    for(uint32_t i = 0; i < ui32DataSize; i += 2, ui32Address += 2)
    {
        if(i < ui32DataSize - 1)
        {
            uintDataBuffer = pData[i];
            uintDataBuffer +=  (uint16_t)(pData[i + 1] << 8);
        }
        else
        {
            uintDataBuffer = pData[i];
            uintDataBuffer += 0xFF00;
        }
        if(HAL_OK !=  HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, ui32Address, (uint64_t)uintDataBuffer))
        {
            HAL_FLASH_Lock();

            return -1;
        }
    }

    HAL_FLASH_Lock();

    return 0;

}

int ReadFlash(uint32_t ui32Address, uint8_t* pData, uint32_t ui32DataSize)
{
    if((ui32Address > FLASH_LAST_SECTOR_END_ADDRESS) || (ui32Address < FLASH_SECTOR_0_START_ADDRESS))
    {
        return -2;
    }

    for(uint32_t i = 0; i < ui32DataSize; i++)
    {
        pData[i] = ((uint8_t*)ui32Address)[i];
    }

    return 0;
}

int EraseSectorByNumber(uint16_t ui16Sector)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t ui32PageError;

    if(ui16Sector < NUMBER_OF_FLASH_SECTORS)
    {
        HAL_FLASH_Unlock();

        EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.PageAddress = ui16Sector * SECTOR_SIZE;
        EraseInitStruct.NbPages = 2;

        if (HAL_FLASHEx_Erase(&EraseInitStruct, &ui32PageError) != HAL_OK)
        {
            HAL_FLASH_Lock();

            return -1;
        }

        while(HAL_OK != FLASH_WaitForLastOperation((uint32_t)50000)){};

        HAL_FLASH_Lock();

        return 0;
    }

    // Sector does not exist
    return -2;
}

int EraseSectorByAddress(uint32_t ui32Address)
{
    volatile uint32_t ui32SectorNumber = (ui32Address * NUMBER_OF_FLASH_SECTORS) / (FLASH_LAST_SECTOR_END_ADDRESS - FLASH_SECTOR_0_START_ADDRESS + 1);

    if((ui32Address > FLASH_LAST_SECTOR_END_ADDRESS) || (ui32Address < FLASH_SECTOR_0_START_ADDRESS))
    {
        return -2;
    }

    return EraseSectorByNumber(ui32SectorNumber);
}

int GetNumOfSectors(void)
{
    return NUMBER_OF_FLASH_SECTORS;
}

void GetSectorStartAdd(uint32_t ui32Sector, uint32_t* pui32Address)
{
    *pui32Address = (ui32Sector * SECTOR_SIZE) + FLASH_SECTOR_0_START_ADDRESS;
}

void GetSectorEndAdd(uint32_t ui32Sector, uint32_t* pui32Address)
{
    *pui32Address = (ui32Sector * SECTOR_SIZE) + (SECTOR_SIZE - 1) + FLASH_SECTOR_0_START_ADDRESS;
}


>>>>>>> 995a20f... Add drivers for the internal flash.
