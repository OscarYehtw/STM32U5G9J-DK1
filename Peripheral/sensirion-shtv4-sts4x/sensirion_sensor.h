/*
 * Unified Sensirion Sensor Driver (SHT4x and STS4x)
 *
 * Copyright (c) 2024, Sensirion AG
 * All rights reserved.
 */

#ifndef SENSIRION_SENSOR_H
#define SENSIRION_SENSOR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* I2C Addresses */
#define SENSIRION_I2C_ADDR_44  0x44
#define SENSIRION_I2C_ADDR_45  0x45
#define SENSIRION_I2C_ADDR_46  0x46

/* Sensor Types */
typedef enum {
    SENSOR_TYPE_STS4X,  // Temperature only
    SENSOR_TYPE_SHT4X   // Temperature + Humidity
} sensirion_sensor_type_t;

/* Measurement Precision */
typedef enum {
    PRECISION_HIGH,
    PRECISION_MEDIUM,
    PRECISION_LOWEST
} sensirion_precision_t;

/* Heater Power (SHT4x only) */
typedef enum {
    HEATER_POWER_HIGHEST,
    HEATER_POWER_MEDIUM,
    HEATER_POWER_LOWEST
} sensirion_heater_power_t;

/* Heater Duration (SHT4x only) */
typedef enum {
    HEATER_DURATION_LONG,   // 1s
    HEATER_DURATION_SHORT   // 0.1s
} sensirion_heater_duration_t;

/* Sensor Configuration */
typedef struct {
    uint8_t i2c_address;
    sensirion_sensor_type_t type;
    void* i2c_hal_ctx;  // Optional: for different I2C buses
} sensirion_sensor_t;

/**
 * Initialize sensor
 * 
 * @param sensor Sensor configuration structure
 * @param i2c_address I2C address of the sensor
 * @param type Sensor type (STS4X or SHT4X)
 * @return 0 on success, error code otherwise
 */
int16_t sensirion_sensor_init(sensirion_sensor_t* sensor, 
                              uint8_t i2c_address,
                              sensirion_sensor_type_t type);

/**
 * Measure temperature (and humidity for SHT4x)
 * 
 * @param sensor Sensor configuration
 * @param precision Measurement precision
 * @param temperature Temperature in milli degrees celsius
 * @param humidity Humidity in milli percent RH (NULL if not needed or STS4x)
 * @return 0 on success, error code otherwise
 */
int16_t sensirion_sensor_measure(sensirion_sensor_t* sensor,
                                 sensirion_precision_t precision,
                                 int32_t* temperature,
                                 int32_t* humidity);

/**
 * Measure with heater (SHT4x only)
 * 
 * @param sensor Sensor configuration (must be SHT4X type)
 * @param power Heater power level
 * @param duration Heater duration
 * @param temperature Temperature in milli degrees celsius
 * @param humidity Humidity in milli percent RH
 * @return 0 on success, error code otherwise
 */
int16_t sensirion_sensor_measure_with_heater(sensirion_sensor_t* sensor,
                                             sensirion_heater_power_t power,
                                             sensirion_heater_duration_t duration,
                                             int32_t* temperature,
                                             int32_t* humidity);

/**
 * Read serial number
 * 
 * @param sensor Sensor configuration
 * @param serial_number Serial number output
 * @return 0 on success, error code otherwise
 */
int16_t sensirion_sensor_serial_number(sensirion_sensor_t* sensor,
                                       uint32_t* serial_number);

/**
 * Soft reset sensor
 * 
 * @param sensor Sensor configuration
 * @return 0 on success, error code otherwise
 */
int16_t sensirion_sensor_soft_reset(sensirion_sensor_t* sensor);

#ifdef __cplusplus
}
#endif

#endif  // SENSIRION_SENSOR_H
