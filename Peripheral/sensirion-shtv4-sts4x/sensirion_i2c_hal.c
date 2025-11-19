/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "sensirion_i2c_hal.h"
#include "stm32u5x9j_discovery_bus.h"
#include "stm32u5xx_hal.h"

/**
 * I2C bus handles from BSP
 */
extern I2C_HandleTypeDef hbus_i2c6;  /* SHTV4 on I2C6 */
extern I2C_HandleTypeDef hbus_i2c2;  /* STS40 on I2C2 */

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication.
 */
void sensirion_i2c_hal_init(void) {
    BSP_I2C6_Init();  /* SHTV4 on I2C6 */
    BSP_I2C2_Init();  /* STS40 on I2C2 */
}

/**
 * Release all resources initialized by sensirion_i2c_hal_init().
 */
void sensirion_i2c_hal_free(void) {
    BSP_I2C6_DeInit();
    BSP_I2C2_DeInit();
}

/**
 * SHT4x specific I2C read (use I2C6)
 */
int8_t sht4x_i2c_read(uint8_t address, uint8_t* data, uint8_t count) {
    return (int8_t)HAL_I2C_Master_Receive(&hbus_i2c6, (uint16_t)(address << 1),
                                          data, count, 100);
}

/**
 * SHT4x specific I2C write (use I2C6)
 */
int8_t sht4x_i2c_write(uint8_t address, const uint8_t* data, uint8_t count) {
    return (int8_t)HAL_I2C_Master_Transmit(&hbus_i2c6, (uint16_t)(address << 1),
                                           (uint8_t*)data, count, 100);
}

/**
 * STS4x specific I2C read (use I2C2)
 */
int8_t sts4x_i2c_read(uint8_t address, uint8_t* data, uint8_t count) {
    return (int8_t)HAL_I2C_Master_Receive(&hbus_i2c2, (uint16_t)(address << 1),
                                          data, count, 100);
}

/**
 * STS4x specific I2C write (use I2C2)
 */
int8_t sts4x_i2c_write(uint8_t address, const uint8_t* data, uint8_t count) {
    return (int8_t)HAL_I2C_Master_Transmit(&hbus_i2c2, (uint16_t)(address << 1),
                                           (uint8_t*)data, count, 100);
}

/**
 * Generic I2C functions for sensirion_i2c.c (should not be used directly)
 * These exist only for linking sensirion_i2c.o
 * All sensor drivers use sensor-specific functions via macro redirection
 */
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
    /* This should never be called - sensors use their specific functions */
    return -1;
}

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint8_t count) {
    /* This should never be called - sensors use their specific functions */
    return -1;
}

/**
 * Sleep for a given number of microseconds. The function should delay the
 * execution for at least the given time, but may also sleep longer.
 *
 * @param useconds the sleep time in microseconds
 */
void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    uint32_t msec = useconds / 1000;
    if (useconds % 1000 > 0) {
        msec++;
    }

    HAL_Delay(msec);
}
