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

static uint32_t l_CRC;

/* CRC handler declaration */
CRC_HandleTypeDef   CrcHandle;

int Init()
{

}

int Reset()
{

}

int CalcSingle(uint8_t* ui8StartAddress, uint32_t ui32DataSize, uint32_t* pui32CRC)
{
   *pui32CRC = 2;
   return 0;
}

int CalcAccumulate(uint8_t* ui8StartAddress, uint32_t ui32DataSize, uint32_t* pui32CRC)
{
    *pui32CRC = 3;
    return 0;
}

