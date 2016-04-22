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


