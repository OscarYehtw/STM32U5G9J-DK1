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
 * Use existing I2C instance from BSP
 */
extern I2C_HandleTypeDef hbus_i2c4;
static I2C_HandleTypeDef *hi2c = &hbus_i2c4;

/**
 * Initialize all hard- and software components that are needed for the I2C
 * communication. (Using existing BSP I2C, no initialization needed)
 */
void sensirion_i2c_hal_init(void) {
    /* sht4x temperature and humidity sensor */
    BSP_I2C6_Init();
    /* sts4x temperature sensor */
    BSP_I2C2_Init();
}

/**
 * Release all resources initialized by sensirion_i2c_hal_init().
 */
void sensirion_i2c_hal_free(void) {
    BSP_I2C6_DeInit();
    BSP_I2C2_DeInit();
}

/**
 * Execute one read transaction on the I2C bus, reading a given number of bytes.
 * If the device does not acknowledge the read command, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to read from
 * @param data    pointer to the buffer where the data is to be stored
 * @param count   number of bytes to read from I2C and store in the buffer
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint8_t count) {
    return (int8_t)HAL_I2C_Master_Receive(hi2c, (uint16_t)(address << 1),
                                          data, count, 100);
}

/**
 * Execute one write transaction on the I2C bus, sending a given number of
 * bytes. The bytes in the supplied buffer must be sent to the given address. If
 * the slave device does not acknowledge any of the bytes, an error shall be
 * returned.
 *
 * @param address 7-bit I2C address to write to
 * @param data    pointer to the buffer containing the data to write
 * @param count   number of bytes to read from the buffer and send over I2C
 * @returns 0 on success, error code otherwise
 */
int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data,
                               uint8_t count) {
    return (int8_t)HAL_I2C_Master_Transmit(hi2c, (uint16_t)(address << 1),
                                           (uint8_t*)data, count, 100);
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
