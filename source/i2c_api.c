/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2014, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#include "mbed_assert.h"
#include "i2c_api.h"

#if DEVICE_I2C

#include "cmsis.h"
#include "pinmap.h"
#include "PeripheralPins.h"

/* Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will
   not remain stuck if the I2C communication is corrupted. */
#define FLAG_TIMEOUT ((int)0x1000)
#define LONG_TIMEOUT ((int)0x8000)

int i2c1_inited = 0;
int i2c2_inited = 0;
int i2c3_inited = 0;

// See I2CSlave.h
#define NoData         0 // the slave has not been addressed
#define ReadAddressed  1 // the master has requested a read from this slave (slave = transmitter)
#define WriteGeneral   2 // the master is writing to all slave
#define WriteAddressed 3 // the master is writing to this slave (slave = receiver)

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef I2cHandle;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

/* I2C event callbacks */
event_cb_t g_cb_s_rx = NULL; // callback for Slave reception completed event
event_cb_t g_cb_s_tx = NULL; // callback for Slave transmission completed event
event_cb_t g_cb_m_rx = NULL; // callback for Master reception completed event
event_cb_t g_cb_m_tx = NULL; // callback for Master transmission completed event
event_cb_t g_cb_e_addr = NULL; // callback for Slave Address Match event
event_cb_t g_cb_e_erro = NULL; // callback for I2C errors event

void i2c_init(i2c_t *obj, PinName sda, PinName scl)
{
    // Determine the I2C to use
    I2CName i2c_sda = (I2CName)pinmap_peripheral(sda, PinMap_I2C_SDA);
    I2CName i2c_scl = (I2CName)pinmap_peripheral(scl, PinMap_I2C_SCL);

    obj->i2c = (I2CName)pinmap_merge(i2c_sda, i2c_scl);
    MBED_ASSERT(obj->i2c != (I2CName)NC);

    // Enable I2C1 clock and pinout if not done
    if ((obj->i2c == I2C_1) && !i2c1_inited) {
        i2c1_inited = 1;
        __HAL_RCC_I2C1_CONFIG(RCC_I2C1CLKSOURCE_SYSCLK);
        __I2C1_CLK_ENABLE();
        // Configure I2C pins
        pinmap_pinout(sda, PinMap_I2C_SDA);
        pinmap_pinout(scl, PinMap_I2C_SCL);
        pin_mode(sda, OpenDrain);
        pin_mode(scl, OpenDrain);
    }
    // Enable I2C2 clock and pinout if not done
    if ((obj->i2c == I2C_2) && !i2c2_inited) {
        i2c2_inited = 1;
        __I2C2_CLK_ENABLE();
        // Configure I2C pins
        pinmap_pinout(sda, PinMap_I2C_SDA);
        pinmap_pinout(scl, PinMap_I2C_SCL);
        pin_mode(sda, OpenDrain);
        pin_mode(scl, OpenDrain);
    }
    // Enable I2C3 clock and pinout if not done

#ifdef I2C3_BASE
    if ((obj->i2c == I2C_3) && !i2c3_inited) {
        i2c3_inited = 1;
        __I2C3_CLK_ENABLE();
        // Configure I2C pins
        pinmap_pinout(sda, PinMap_I2C_SDA);
        pinmap_pinout(scl, PinMap_I2C_SCL);
        pin_mode(sda, OpenDrain);
        pin_mode(scl, OpenDrain);
    }
#endif
    // Reset to clear pending flags if any
    i2c_reset(obj);

    // I2C configuration
    i2c_frequency(obj, 100000); // 100 kHz per default

    // I2C master by default
    obj->slave = 0;
}

void i2c_frequency(i2c_t *obj, int hz)
{
    MBED_ASSERT((hz == 100000) || (hz == 400000) || (hz == 1000000));
    I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);
    int timeout;
    uint32_t tim;

    // wait before init
    timeout = LONG_TIMEOUT;
    while ((__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_BUSY)) && (timeout-- != 0));

    switch (hz) {
		case 100000:
			tim = 0x00201D2B; // Standard mode
			break;
		case 200000:
			tim = 0x0010021E; // Fast mode
			break;
		case 400000:
			tim = 0x0010020A; // Fast mode
			break;
		default:
			tim = 0x00201D2B; // Standard mode as default
			break;
    }

    // I2C configuration
    I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    I2cHandle.Init.OwnAddress1     = 0;
    I2cHandle.Init.OwnAddress2     = 0;
    I2cHandle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    I2cHandle.Init.Timing 		    = tim; //@mor Calculated with excel tool provided in ST AN4235

    HAL_I2C_Init(&I2cHandle);
    if (obj->slave) {
        /* Enable Address Acknowledge */
    	I2cHandle.Instance->CR2 &= ~I2C_CR2_NACK;
    }

}

inline int i2c_start(i2c_t *obj)
{
    I2C_TypeDef *i2c = (I2C_TypeDef *)(obj->i2c);
    int timeout;

    I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);

    // Clear Acknowledge failure flag
    __HAL_I2C_CLEAR_FLAG(&I2cHandle, I2C_FLAG_AF);

    // Generate the START condition
    i2c->CR2 |= I2C_CR2_START; //@mor according to specs the start bit is in the CR2 for the
    #warning "i2c->CR1 |= I2C_CR1_START;"

    // Wait the START condition has been correctly sent
    //@mor is this part necessary???
    //#mab:
     timeout = FLAG_TIMEOUT;
     while (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_BUSY) == RESET) {
         if ((timeout--) == 0) {
             return 1;
         }
     }

    return 0;
}

inline int i2c_stop(i2c_t *obj)
{
    I2C_TypeDef *i2c = (I2C_TypeDef *)(obj->i2c);

    // Generate the STOP condition
    //i2c->CR1 |= I2C_CR1_STOP; //@mor
    i2c->CR2 |= I2C_CR2_STOP;

    return 0;
}

int i2c_read(i2c_t *obj, int address, char *data, int length, int stop)
{
    I2C_TypeDef *i2c = (I2C_TypeDef *)(obj->i2c);
    I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);
    int timeout;
    int count;
    int value;

    //@mor from mbed2 i2c_api.c for nucleo f072
    /* update CR2 register */
    i2c->CR2 = (i2c->CR2 & (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP)))
                | (uint32_t)(((uint32_t)address & I2C_CR2_SADD) | (((uint32_t)length << 16) & I2C_CR2_NBYTES) | (uint32_t)I2C_SOFTEND_MODE | (uint32_t)I2C_GENERATE_START_READ);

    // Read all bytes
    for (count = 0; count < length; count++) {
    	value = i2c_byte_read(obj, 0);
    	data[count] = (char)value;
    }
    //@mor end

    // Wait until transfer is complete
    //@mor I2C_FLAG_SB -> I2C_FLAG_TC
    timeout = FLAG_TIMEOUT;
     while (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TC) == RESET) {
         timeout--;
         if (timeout == 0) {
             return -1;
         }
     }
     __HAL_I2C_CLEAR_FLAG(&I2cHandle, I2C_FLAG_TC);

     // If not repeated start, send stop.
     if (stop) {
    	 i2c_stop(obj);
    	 /* Wait until STOPF flag is set */
    timeout = FLAG_TIMEOUT;
    	 while (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_STOPF) == RESET) {
        timeout--;
        if (timeout == 0) {
            return -1;
        }
    }
    	 /* Clear STOP Flag */
    	 __HAL_I2C_CLEAR_FLAG(&I2cHandle, I2C_FLAG_STOPF);
     }


    //i2c->RXDR = __HAL_I2C_7BIT_ADD_READ(address); //@mor

     //@mor looks like the address is already provided when they write the CR2
     //also the behavior is different.
    // Wait address is acknowledged
//    timeout = FLAG_TIMEOUT;
//    while (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_ADDR) == RESET) {
//        timeout--;
//        if (timeout == 0) {
//            return -1;
//        }
//    }
    //__HAL_I2C_CLEAR_ADDRFLAG(&I2cHandle);
    //__HAL_I2C_CLEAR_FLAG(&I2cHandle, I2C_FLAG_ADDR);

    // Read all bytes except last one
//    for (count = 0; count < (length - 1); count++) {
//        value = i2c_byte_read(obj, 0);
//        data[count] = (char)value;
//    }

    // If not repeated start, send stop.
    // Warning: must be done BEFORE the data is read.
//    if (stop) {
//        i2c_stop(obj);
//    }
//
//    // Read the last byte
//    value = i2c_byte_read(obj, 1);
//    data[count] = (char)value;

    return length;
}

int i2c_write(i2c_t *obj, int address, const char *data, int length, int stop)
{
    I2C_TypeDef *i2c = (I2C_TypeDef *)(obj->i2c);
    I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);
    int timeout;
    int count;

    /* update CR2 register */ //@mor
    i2c->CR2 = (i2c->CR2 & (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_STOP)))
               | (uint32_t)(((uint32_t)address & I2C_CR2_SADD) | (((uint32_t)length << 16) & I2C_CR2_NBYTES) | (uint32_t)I2C_SOFTEND_MODE | (uint32_t)I2C_GENERATE_START_WRITE);

    //Write all the data
    for (count = 0; count < length; count++) {
    	i2c_byte_write(obj, data[count]);
    }

    // Wait transfer complete
    timeout = FLAG_TIMEOUT;
    while (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TC) == RESET) {
    	timeout--;
    	if (timeout == 0) {
    		return -1;
    	}
    }
    __HAL_I2C_CLEAR_FLAG(&I2cHandle, I2C_FLAG_TC);

    // If not repeated start, send stop.
    if (stop) {
    	i2c_stop(obj);
    	/* Wait until STOPF flag is set */
    	timeout = FLAG_TIMEOUT;
    	while (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_STOPF) == RESET) {
    		timeout--;
    		if (timeout == 0) {
    			return -1;
    		}
    	}
    	/* Clear STOP Flag */
    	__HAL_I2C_CLEAR_FLAG(&I2cHandle, I2C_FLAG_STOPF);
    }

    //i2c_start(obj); //@mor

    // Wait until SB flag is set
//    timeout = FLAG_TIMEOUT;
//    while (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_SB) == RESET) {
//        timeout--;
//        if (timeout == 0) {
//            return -1;
//        }
//    }
    //
    //i2c->TXDR = __HAL_I2C_7BIT_ADD_WRITE(address); //@mor

/*//@mor the procedure is different for the m0
    // Wait address is acknowledged
    timeout = FLAG_TIMEOUT;
    while (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_ADDR) == RESET) {
        timeout--;
        if (timeout == 0) {
            return -1;
        }
    }
    //__HAL_I2C_CLEAR_ADDRFLAG(&I2cHandle);
    __HAL_I2C_CLEAR_FLAG(&I2cHandle, I2C_FLAG_ADDR);

    for (count = 0; count < length; count++) {
        if (i2c_byte_write(obj, data[count]) != 1) {
            i2c_stop(obj);
            return -1;
        }
    }

    // If not repeated start, send stop.
    if (stop) {
        i2c_stop(obj);
    }
*/
    return count;
}

int i2c_byte_read(i2c_t *obj, int last)
{
    I2C_TypeDef *i2c = (I2C_TypeDef *)(obj->i2c);
    int timeout;

#warning "jagomo: ack generation"
//    if (last) {
//        // Don't acknowledge the last byte
//        i2c->CR1 &= ~I2C_CR1_ACK;
//    } else {
//        // Acknowledge the byte
//        i2c->CR1 |= I2C_CR1_ACK;
//    }

    // Wait until the byte is received
    timeout = FLAG_TIMEOUT;
    while (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_RXNE) == RESET) {
        if ((timeout--) == 0) {
            return -1;
        }
    }

    return (int)i2c->RXDR;
}

int i2c_byte_write(i2c_t *obj, int data)
{
    I2C_TypeDef *i2c = (I2C_TypeDef *)(obj->i2c);
    int timeout;

    // Wait until the previous byte is transmitted
    timeout = FLAG_TIMEOUT;
    //@mor I2C_FLAG_TXE -> I2C_FLAG_TXIS
    while ((__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TXIS) == RESET) /*&&
            (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_BTF) == RESET)*/ /* #mab*/) {
        if ((timeout--) == 0) {
            return 0;
        }
    }

    i2c->TXDR = (uint8_t)data;

    return 1;
}

void i2c_reset(i2c_t *obj)
{
    int timeout;

    // wait before reset
    timeout = LONG_TIMEOUT;
    while ((__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_BUSY)) && (timeout-- != 0));

    if (obj->i2c == I2C_1) {
        __I2C1_FORCE_RESET();
        __I2C1_RELEASE_RESET();
    }
    if (obj->i2c == I2C_2) {
        __I2C2_FORCE_RESET();
        __I2C2_RELEASE_RESET();
    }
#ifdef I2C3_BASE
    if (obj->i2c == I2C_3) {
        __I2C3_FORCE_RESET();
        __I2C3_RELEASE_RESET();
    }
#endif
}

void i2c_set_own_address(i2c_t *obj, uint32_t address)
{
	I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);
    uint16_t tmpreg = 0;

    // Get the old register value
    tmpreg = I2cHandle.Instance->OAR1;
    // Reset address bits
    tmpreg &= 0xFC00;
    // Set new address
    tmpreg |= (uint16_t)((uint16_t)address & (uint16_t)0x00FE); // 7-bits
    // Store the new register value
    I2cHandle.Instance->OAR1 = tmpreg;
    // Own address 1 enable
    I2cHandle.Instance->OAR1 |= I2C_OAR1_OA1EN;
//    if(tmpreg != 0)
//    {
//       if(I2cHandle.Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT)
//       {
//    	   I2cHandle.Instance->OAR1 = (I2C_OAR1_OA1EN | tmpreg);
//       }
//    }
}

#if DEVICE_I2CSLAVE

void i2c_slave_address(i2c_t *obj, int idx, uint32_t address, uint32_t mask)
{
	I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);
    uint16_t tmpreg = 0;

    // Get the old register value
    tmpreg = I2cHandle.Instance->OAR1;
    // Reset address bits
    tmpreg &= 0xFC00;
    // Set new address
    tmpreg |= (uint16_t)((uint16_t)address & (uint16_t)0x00FE); // 7-bits
    // Store the new register value
    I2cHandle.Instance->OAR1 = tmpreg;
}

void i2c_slave_mode(i2c_t *obj, int enable_slave)
{
    I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);
    if (enable_slave) {
        obj->slave = 1;
        /* Enable Address Acknowledge */
        I2cHandle.Instance->CR1 |= I2C_CR1_ACK;
    }
    else
    {
    	I2cHandle.Instance->CR1 &= ~I2C_CR1_ACK;
    	obj->slave = 0;
    }
}

int i2c_slave_receive(i2c_t *obj)
{
    int retValue = NoData;
    I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);

    if (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_BUSY) == 1) {
        if (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_ADDR) == 1) {
            if (__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_TRA) == 1)
                retValue = ReadAddressed;
            else
                retValue = WriteAddressed;

            __HAL_I2C_CLEAR_FLAG(&I2cHandle, I2C_FLAG_ADDR);
        }
    }

    return (retValue);
}

int i2c_slave_read(i2c_t *obj, char *data, int length)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	uint32_t timeout;
	timeout = FLAG_TIMEOUT;

	I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);

	status = HAL_I2C_Slave_Receive(&I2cHandle, data, length, timeout);

	return status;
}

int i2c_slave_write(i2c_t *obj, const char *data, int length)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	uint32_t timeout;
	timeout = FLAG_TIMEOUT;

	I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);

	status = HAL_I2C_Slave_Transmit(&I2cHandle, data, length, timeout);

	return status;
}
#endif // DEVICE_I2CSLAVE

#ifdef DEVICE_I2C_DMA
void i2c_register_event_cb(
		event_cb_t cb_s_rx,
		event_cb_t cb_s_tx,
		event_cb_t cb_m_rx,
		event_cb_t cb_m_tx,
		event_cb_t cb_e_addr,
		event_cb_t cb_e_erro)
{
	if(cb_s_rx)	g_cb_s_rx = cb_s_rx;
	if(cb_s_tx)	g_cb_s_tx = cb_s_tx;
	if(cb_m_rx)	g_cb_m_rx = cb_m_rx;
	if(cb_m_tx)	g_cb_m_tx = cb_m_tx;
	if(cb_e_addr) g_cb_e_addr = cb_e_addr;
	if(cb_e_erro) g_cb_e_erro = cb_e_erro;
}

int i2c_enable_i2c_it(i2c_t *obj)
{
	I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);

	if(I2cHandle.State == HAL_I2C_STATE_READY)
	{
		if(__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_BUSY) == SET)
		{
			return HAL_BUSY;
		}

		/* Process Locked */
		__HAL_LOCK(&I2cHandle);

		I2cHandle.ErrorCode = HAL_I2C_ERROR_NONE;

		/* Enable Address Acknowledge */
		I2cHandle.Instance->CR2 &= ~I2C_CR2_NACK;

		/* Process Unlocked */
		__HAL_UNLOCK(&I2cHandle);

		/* Note : The I2C interrupts must be enabled after unlocking current process
				  to avoid the risk of I2C interrupt handle execution before current
				  process unlock */

		/* Enable EVT, BUF and ERR interrupt | I2C_IT_BUF */
		__HAL_I2C_ENABLE_IT(&I2cHandle, I2C_IT_ADDRI | I2C_IT_ERRI | I2C_IT_STOPI);

		return HAL_OK;
	}
	else
	{
	   return HAL_BUSY;
	}
}

int i2c_master_transmit_DMA(i2c_t *obj, int address, const unsigned char *data, int length, char stop)
{
	HAL_StatusTypeDef status = HAL_ERROR;
	I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);
	status = HAL_I2C_Master_Transmit_DMA(&I2cHandle, address, data, length, stop);

	return status;
}

int i2c_master_receive_DMA(i2c_t *obj, int address, unsigned char *data, int length, char stop)
{
	HAL_StatusTypeDef status = HAL_ERROR;

	I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);
	status = HAL_I2C_Master_Receive_DMA(&I2cHandle, address, data, length, stop);

	return status;
}

int i2c_slave_transmit_DMA(i2c_t *obj, const unsigned char *data, int length)
{
	HAL_StatusTypeDef status = HAL_ERROR;

	I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);
	status = HAL_I2C_Slave_Transmit_DMA(&I2cHandle, data, length);

	return status;
}

int i2c_slave_receive_DMA(i2c_t *obj, unsigned char *data, int length)
{
	HAL_StatusTypeDef status = HAL_ERROR;

	I2cHandle.Instance = (I2C_TypeDef *)(obj->i2c);
	status = HAL_I2C_Slave_Receive_DMA(&I2cHandle, data, length);

	return status;
}

/**
  * Initializes the Global MSP.
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
	{
		/* DMA controller clock enable */
		__DMA1_CLK_ENABLE();

		hdma_i2c1_rx.Instance = DMA1_Channel3;
		hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
		hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		HAL_DMA_Init(&hdma_i2c1_rx);

		__HAL_DMA1_REMAP(HAL_DMA1_CH3_I2C1_RX);

		__HAL_LINKDMA(hi2c,hdmarx,hdma_i2c1_rx);

		hdma_i2c1_tx.Instance = DMA1_Channel2;
		hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
		hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		HAL_DMA_Init(&hdma_i2c1_tx);

		__HAL_DMA1_REMAP(HAL_DMA1_CH2_I2C1_TX);

		__HAL_LINKDMA(hi2c,hdmatx,hdma_i2c1_tx);

		/* DMA interrupt init */
		HAL_NVIC_SetPriority(DMA1_Ch2_3_DMA2_Ch1_2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(DMA1_Ch2_3_DMA2_Ch1_2_IRQn);

		/* I2C interrupt init */
		HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(I2C1_IRQn);
	}
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c)
{
	if(hi2c->Instance == I2C1)
	{
		/* Peripheral clock disable */
		__I2C1_CLK_DISABLE();

		/* Peripheral interrupt DeInit*/
		HAL_NVIC_DisableIRQ(I2C1_IRQn);

		/**I2C1 GPIO Configuration
		PB8     ------> I2C1_SCL
		PB9     ------> I2C1_SDA
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

		/* Peripheral DMA DeInit*/
    	HAL_DMA_DeInit(hi2c->hdmarx);
    	HAL_DMA_DeInit(hi2c->hdmatx);
	}
}

/**
* @brief This function handles DMA1 channel 2 to 3 and DMA2 channel 1 to 2 interrupts.
*/
void DMA1_Ch2_3_DMA2_Ch1_2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
}

/**
* @brief This function handles I2C1 global interrupt (combined with EXTI line 23 interrupt).
*/
void I2C1_IRQHandler(void)
{
	uint16_t dataCnt = 0;

	if (I2cHandle.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
		HAL_I2C_ER_IRQHandler(&I2cHandle);
	} else {
		/* I2C Slave received Stop */
		if ((__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_STOPF) == SET) &&
		   (__HAL_I2C_GET_IT_SOURCE(&I2cHandle, I2C_IT_STOPI) == SET))
		{
			if(I2cHandle.State == HAL_I2C_STATE_SLAVE_BUSY_RX)
			{
				/* Calculate number of received bytes */
				dataCnt = I2C_DATA_LENGTH_MAX - (hdma_i2c1_rx.Instance->CNDTR);
				/* Set actual received size */
				I2cHandle.XferSize = dataCnt;

				/* Disable DMA Request */
				I2cHandle.Instance->CR1 &= ~I2C_CR1_RXDMAEN;

				/* Disable DMA RX Channel */
				HAL_DMA_Abort(&hdma_i2c1_rx);

				/* Configure DMA Stream data length */
				//hdma_i2c1_rx.Instance->CNDTR = I2C_DATA_LENGTH_MAX;
			}
			/* Clear STOP Flag */
			__HAL_I2C_CLEAR_FLAG(&I2cHandle, I2C_FLAG_STOPF);
			I2cHandle.State = HAL_I2C_STATE_READY;
		}
		/* I2C Slave address match  ---------------------------------------------------*/
		if ((__HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_ADDR) == SET) &&
		   (__HAL_I2C_GET_IT_SOURCE(&I2cHandle, I2C_IT_ADDRI) == SET))
		{
			I2cHandle.tDir = __HAL_I2C_GET_FLAG(&I2cHandle, I2C_FLAG_DIR);
			/* Clear ADDR flag */
			__HAL_I2C_CLEAR_FLAG(&I2cHandle, I2C_FLAG_ADDR);
			/* Call Address Matched callback */
			g_cb_e_addr(&I2cHandle);
		}
  }
}

/**
  * @brief  Master Tx Transfer completed callback.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	g_cb_m_tx(hi2c);
}

/**
  * @brief  Master Rx Transfer completed callback.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	g_cb_m_rx(hi2c);
}

/** @brief  Slave Tx Transfer completed callback.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	g_cb_s_tx(hi2c);
}

/**
  * @brief  Slave Rx Transfer completed callback.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	g_cb_s_rx(hi2c);
}

/**
  * @brief  I2C error callback.
  * @param  hi2c: pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	g_cb_e_erro(hi2c);
}

#endif //DEVICE_I2C_DMA

#endif // DEVICE_I2C
