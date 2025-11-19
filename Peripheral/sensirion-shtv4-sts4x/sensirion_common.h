/*
 * Common utilities for SHT4x and STS4x sensor drivers
 *
 * Copyright (c) 2024, Sensirion AG
 * All rights reserved.
 */

#ifndef SENSIRION_COMMON_H
#define SENSIRION_COMMON_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Common I2C commands (same for both SHT4x and STS4x) */
#define SENSIRION_CMD_MEASURE_HIGH_PRECISION    0xFD
#define SENSIRION_CMD_MEASURE_MEDIUM_PRECISION  0xF6
#define SENSIRION_CMD_MEASURE_LOWEST_PRECISION  0xE0
#define SENSIRION_CMD_SERIAL_NUMBER             0x89
#define SENSIRION_CMD_SOFT_RESET                0x94

/* SHT4x specific heater commands */
#define SENSIRION_CMD_HEATER_HIGHEST_LONG       0x39
#define SENSIRION_CMD_HEATER_HIGHEST_SHORT      0x32
#define SENSIRION_CMD_HEATER_MEDIUM_LONG        0x2F
#define SENSIRION_CMD_HEATER_MEDIUM_SHORT       0x24
#define SENSIRION_CMD_HEATER_LOWEST_LONG        0x1E
#define SENSIRION_CMD_HEATER_LOWEST_SHORT       0x15

/* Measurement delays (microseconds) */
#define SENSIRION_DELAY_HIGH_PRECISION          10000
#define SENSIRION_DELAY_MEDIUM_PRECISION        5000
#define SENSIRION_DELAY_LOWEST_PRECISION        2000
#define SENSIRION_DELAY_HEATER_LONG             1100000
#define SENSIRION_DELAY_HEATER_SHORT            110000
#define SENSIRION_DELAY_SOFT_RESET              10000

/**
 * Helper functions to convert bytes to integers
 */
static inline uint16_t sensirion_common_bytes_to_uint16_t(const uint8_t* bytes) {
    return (uint16_t)bytes[0] << 8 | (uint16_t)bytes[1];
}

static inline uint32_t sensirion_common_bytes_to_uint32_t(const uint8_t* bytes) {
    return (uint32_t)bytes[0] << 24 | (uint32_t)bytes[1] << 16 |
           (uint32_t)bytes[2] << 8 | (uint32_t)bytes[3];
}

/**
 * convert_ticks_to_celsius() - convert temperature ticks to physical temperature
 *
 * @param ticks temperature in ticks
 * @return Temperature in milli centigrade
 *
 * Formula: T[°C] = -45 + 175 * ticks / 65535
 * Optimized: ((21875 * ticks) >> 13) - 45000
 */
static inline int32_t sensirion_convert_ticks_to_celsius(uint16_t ticks) {
    return ((21875 * (int32_t)ticks) >> 13) - 45000;
}

/**
 * convert_ticks_to_percent_rh() - convert humidity ticks to physical humidity
 *
 * @param ticks relative humidity in ticks
 * @return Humidity in milli percent relative humidity
 *
 * Formula: RH[%] = -6 + 125 * ticks / 65535
 * Optimized: ((15625 * ticks) >> 13) - 6000
 */
static inline int32_t sensirion_convert_ticks_to_percent_rh(uint16_t ticks) {
    return ((15625 * (int32_t)ticks) >> 13) - 6000;
}

/**
 * Generic measure function for sensors
 * 
 * @param i2c_address I2C address of the sensor
 * @param command Command byte to send
 * @param delay_us Delay in microseconds after sending command
 * @param buffer Buffer for communication (must be at least read_bytes size)
 * @param read_bytes Number of bytes to read back
 * @return 0 on success, error code otherwise
 */
int16_t sensirion_measure_ticks(uint8_t i2c_address, uint8_t command, 
                                uint32_t delay_us, uint8_t* buffer, 
                                uint8_t read_bytes);

/**
 * Read serial number from sensor
 * 
 * @param i2c_address I2C address of the sensor
 * @param serial_number Pointer to store the serial number
 * @return 0 on success, error code otherwise
 */
int16_t sensirion_read_serial_number(uint8_t i2c_address, uint32_t* serial_number);

/**
 * Perform soft reset on sensor
 * 
 * @param i2c_address I2C address of the sensor
 * @return 0 on success, error code otherwise
 */
int16_t sensirion_soft_reset(uint8_t i2c_address);

#ifdef __cplusplus
}
#endif

#endif  // SENSIRION_COMMON_H
