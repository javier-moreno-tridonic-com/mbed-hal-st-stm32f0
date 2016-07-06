/*************************************************************************//**
 *
 * \file    crc_api.c
 * \brief   ...
 *
 * \note    None
 *
 * \author  Daniel Cesarini <daniel.cesarini@tridonic.com>
 * \author  Martin Tomasini <martin.tomasini2@tridonic.com>
 * \version 0.1
 * \date    4 July 2016
 * \bug     None
 *
 * \copyright 2016, Tridonic GmbH & Co KG
 *                  Färbergasse 15 - 6851 Dornbirn, Austria
 *
 *****************************************************************************/

#include "crc_api.h"
#include "cmsis.h"

#define __bswap_constant_32(x) \
  ((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) | \
   (((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24))

#define __bswap_32(x) __bswap_constant_32 (x)

static uint32_t l_CRC;

/* CRC handler declaration */
CRC_HandleTypeDef   CRC_HandleStruct;

int Init()
{
    __HAL_RCC_CRC_CLK_ENABLE();

    CRC_HandleStruct.Instance = CRC;
    CRC_HandleStruct.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;       // use default poly=0x04c11db7
    CRC_HandleStruct.Init.CRCLength               = CRC_POLYLENGTH_32B;              // calc 32bit crc
    CRC_HandleStruct.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;       // use default init value (0xffffffff)
    CRC_HandleStruct.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_BYTE;    // The input data are inverted
    CRC_HandleStruct.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE; // The output data are inverted
    //CRC_HandleStruct.InputDataFormat              = CRC_INPUTDATA_FORMAT_BYTES;      // The input data are 8 bits length
    CRC_HandleStruct.InputDataFormat              = CRC_INPUTDATA_FORMAT_WORDS;      // The input data are 8 bits length

    HAL_CRC_Init(&CRC_HandleStruct);
}

int Reset()
{
    /* Reset CRC Calculation Unit */
    // TODO: CHECK THIS!!! //     __HAL_CRC_DR_RESET(&CRC_HandleStruct);
    // Needed to clear the CRC and start a new crc calculation

    HAL_CRC_DeInit(&CRC_HandleStruct);
    HAL_CRC_Init(&CRC_HandleStruct);
}

int CalcSingle(uint8_t* ui8StartAddress, uint32_t ui32DataSize, uint32_t* pui32CRC)
{
   *pui32CRC = 2;
   return 0;
}

int CalcAccumulate(uint8_t* pui8StartAddress, uint32_t ui32DataSize, uint32_t* pui32CRC)
{
    //printf("CalcAccumulate: %x, ui32DataSize: %d \n", pui8StartAddress, ui32DataSize);
    uint32_t uwCRCValue;
    //uint32_t *pui32StartAddress = (uint32_t*)pui8StartAddress;
    uint32_t ui32DataSize32Bits = ui32DataSize / 4;
    uint32_t ui32TemporaryDataStorage;
    /*uint8_t* pui8TemporaryArray;
    uint8_t ui8TempByte;*/

    // Reverse input bits
    for (uint32_t i = 0; i < ui32DataSize32Bits; i++)
    {
        //printf("StartAddress: %x, i: %d \n", pui32StartAddress, i);
        ui32TemporaryDataStorage =
                (pui8StartAddress[i*4+0] << 24) +
                (pui8StartAddress[i*4+1] << 16) +
                (pui8StartAddress[i*4+2] <<  8) +
                (pui8StartAddress[i*4+3] <<  0);

                //pui32StartAddress[i];
        //ui32TemporaryDataStorage = __RBIT(ui32TemporaryDataStorage);

        //ui32TemporaryDataStorage = __bswap_32(ui32TemporaryDataStorage);
        //ui32TemporaryDataStorage = __REV(ui32TemporaryDataStorage);   // DECOMMENTED BECAUSE IT IS NOT NEEDED. DISCUSS WITHN mab TODO!!!
        /*
        pui8TemporaryArray = (uint8_t*) &ui32TemporaryDataStorage;
        ui8TempByte = pui8TemporaryArray[0];
        pui8TemporaryArray[0] = pui8TemporaryArray[3];
        pui8TemporaryArray[3] = ui8TempByte;
        ui8TempByte = pui8TemporaryArray[1];
        pui8TemporaryArray[1] = pui8TemporaryArray[2];
        pui8TemporaryArray[2] = ui8TempByte;
        */

        uwCRCValue = HAL_CRC_Accumulate(&CRC_HandleStruct, &ui32TemporaryDataStorage, 1);
    }

    // reverse CRC result before final XOR
    //uwCRCValue = __RBIT(uwCRCValue);
    uwCRCValue ^= 0xffffffff;

    *pui32CRC = uwCRCValue;

    /*// Calculate the CRC with the HAL_CRC_Accumulate function
    uwCRCValue = HAL_CRC_Accumulate(&CRC_HandleStruct, ui8StartAddress, ui32DataSize);*/

    return 0;
}

