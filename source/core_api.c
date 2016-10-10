/***************************************************
*  core_api.c
*
*  
*
*  Tridonic GmbH & Co KG
*  Färbergasse 15
*  6851 Dornbirn, Austria
*
***************************************************/
/* Includes ------------------------------------------------------------------*/
#include "core_api.h"
#include "cmsis.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

void SwReset(void)
{
    HAL_NVIC_SystemReset();
}

void SwitchEndian32(uint32_t* pui32Value)
{
    *pui32Value = __REV(*pui32Value);
}

void SwitchEndian16(uint16_t* pui16Value)
{
    *pui16Value = __REV16(*pui16Value);
}
