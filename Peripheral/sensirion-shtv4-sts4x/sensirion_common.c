/*
 * Common implementation for SHT4x and STS4x sensor drivers
 *
 * Copyright (c) 2024, Sensirion AG
 * All rights reserved.
 */

#include "sensirion_common.h"
#include "sensirion_i2c.h"
#include "sensirion_i2c_hal.h"

int16_t sensirion_measure_ticks(uint8_t i2c_address, uint8_t command, 
                                uint32_t delay_us, uint8_t* buffer, 
                                uint8_t read_bytes) {
    int16_t error;
    uint16_t offset = 0;
    buffer[offset++] = command;

    error = sensirion_i2c_write_data(i2c_address, buffer, offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(delay_us);

    error = sensirion_i2c_read_data_inplace(i2c_address, buffer, read_bytes);
    return error;
}

int16_t sensirion_read_serial_number(uint8_t i2c_address, uint32_t* serial_number) {
    int16_t error;
    uint8_t buffer[6];
    uint16_t offset = 0;
    buffer[offset++] = SENSIRION_CMD_SERIAL_NUMBER;

    error = sensirion_i2c_write_data(i2c_address, buffer, offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(SENSIRION_DELAY_HIGH_PRECISION);

    error = sensirion_i2c_read_data_inplace(i2c_address, buffer, 4);
    if (error) {
        return error;
    }
    *serial_number = sensirion_common_bytes_to_uint32_t(buffer);
    return NO_ERROR;
}

int16_t sensirion_soft_reset(uint8_t i2c_address) {
    int16_t error;
    uint8_t buffer[2];
    uint16_t offset = 0;
    buffer[offset++] = SENSIRION_CMD_SOFT_RESET;

    error = sensirion_i2c_write_data(i2c_address, buffer, offset);
    if (error) {
        return error;
    }
    sensirion_i2c_hal_sleep_usec(SENSIRION_DELAY_SOFT_RESET);
    return NO_ERROR;
}
