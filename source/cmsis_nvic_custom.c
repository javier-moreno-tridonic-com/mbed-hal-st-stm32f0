/* mbed Microcontroller Library
 * CMSIS-style functionality to support dynamic vectors
 *******************************************************************************
 * Copyright (c) 2011 ARM Limited. All rights reserved.
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
 * 3. Neither the name of ARM Limited nor the names of its contributors
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
#include "cmsis-core/cmsis_nvic.h"

#if defined(YOTTA_CFG_CMSIS_NVIC_HAS_CUSTOM_VTOR)

void NVIC_SetVector(IRQn_Type IRQn, uint32_t vector) {
    int i;
    // Space for dynamic vectors, initialised to allocate in R/W
    static volatile uint32_t *vectors = (uint32_t *)NVIC_RAM_VECTOR_ADDRESS;
    
    // Copy and switch to dynamic vectors if first time called
    if ((SYSCFG->CFGR1 & SYSCFG_CFGR1_MEM_MODE) == 0) {
      uint32_t *old_vectors = (uint32_t *)NVIC_FLASH_VECTOR_ADDRESS;
      for (i = 0; i < NVIC_NUM_VECTORS; i++) {
          vectors[i] = old_vectors[i];
      }
      SYSCFG->CFGR1 |= SYSCFG_CFGR1_MEM_MODE; // Embedded SRAM mapped at 0x00000000
    }
    
    vectors[IRQn + NVIC_USER_IRQ_OFFSET] = vector;
}

uint32_t NVIC_GetVector(IRQn_Type IRQn) {
    uint32_t *vectors = (uint32_t*)NVIC_RAM_VECTOR_ADDRESS;
    return vectors[IRQn + NVIC_USER_IRQ_OFFSET];
}

#endif // YOTTA_CFG_CMSIS_NVIC_HAS_CUSTOM_VTOR
