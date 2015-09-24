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

I2C_HandleTypeDef I2cHandle;

int i2c1_inited = 0;
int i2c2_inited = 0;
int i2c3_inited = 0;

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
        __I2C1_CONFIG(RCC_I2C1CLKSOURCE_SYSCLK);
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
/*
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
*/

    switch (hz) {
   		case 100000:
   			tim = 0x10805E89; // Standard mode
   			break;
   		case 200000:
   			tim = 0x00901850; // Fast mode
   			break;
   		case 400000:
   			tim = 0x00700818; // Fast mode
   			break;
   		default:
   			tim = 0x10805E89; // Standard mode as default
   			break;
       }

    // I2C configuration
    I2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    #warning "I2C clock speed"
    //I2cHandle.Init.ClockSpeed      = hz;
    I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    //I2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    I2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    I2cHandle.Init.OwnAddress1     = 0;
    I2cHandle.Init.OwnAddress2     = 0;
    I2cHandle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    I2cHandle.Init.Timing 		    = tim; //@mor Calculated with excel tool provided in ST AN4235

    HAL_I2C_Init(&I2cHandle);
    if (obj->slave) {
        /* Enable Address Acknowledge */
        //I2cHandle.Instance->CR1 |= I2C_CR1_ACK;
        #warning "I2cHandle.Instance->CR1 |= I2C_CR1_ACK;"
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

#endif // DEVICE_I2C
