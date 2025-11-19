/*
 * Unified Sensirion Sensor Driver Implementation (SHT4x and STS4x)
 *
 * Copyright (c) 2024, Sensirion AG
 * All rights reserved.
 */

#include <stddef.h>
#include "sensirion_sensor.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"

/* Command and delay mapping */
static const uint8_t precision_commands[] = {
    SENSIRION_CMD_MEASURE_HIGH_PRECISION,
    SENSIRION_CMD_MEASURE_MEDIUM_PRECISION,
    SENSIRION_CMD_MEASURE_LOWEST_PRECISION
};

static const uint32_t precision_delays[] = {
    SENSIRION_DELAY_HIGH_PRECISION,
    SENSIRION_DELAY_MEDIUM_PRECISION,
    SENSIRION_DELAY_LOWEST_PRECISION
};

static const uint8_t heater_commands[3][2] = {
    {SENSIRION_CMD_HEATER_HIGHEST_LONG, SENSIRION_CMD_HEATER_HIGHEST_SHORT},
    {SENSIRION_CMD_HEATER_MEDIUM_LONG, SENSIRION_CMD_HEATER_MEDIUM_SHORT},
    {SENSIRION_CMD_HEATER_LOWEST_LONG, SENSIRION_CMD_HEATER_LOWEST_SHORT}
};

static const uint32_t heater_delays[] = {
    SENSIRION_DELAY_HEATER_LONG,
    SENSIRION_DELAY_HEATER_SHORT
};

int16_t sensirion_sensor_init(sensirion_sensor_t* sensor, 
                              uint8_t i2c_address,
                              sensirion_sensor_type_t type) {
    if (!sensor) {
        return -1;
    }
    
    sensor->i2c_address = i2c_address;
    sensor->type = type;
    sensor->i2c_hal_ctx = NULL;
    
    return sensirion_sensor_soft_reset(sensor);
}

int16_t sensirion_sensor_measure(sensirion_sensor_t* sensor,
                                 sensirion_precision_t precision,
                                 int32_t* temperature,
                                 int32_t* humidity) {
    if (!sensor || !temperature) {
        return -1;
    }
    
    uint8_t buffer[6];
    uint8_t read_bytes = (sensor->type == SENSOR_TYPE_SHT4X) ? 4 : 2;
    
    int16_t error = sensirion_measure_ticks(
        sensor->i2c_address,
        precision_commands[precision],
        precision_delays[precision],
        buffer,
        read_bytes
    );
    
    if (error) {
        return error;
    }
    
    uint16_t temp_ticks = sensirion_common_bytes_to_uint16_t(buffer);
    *temperature = sensirion_convert_ticks_to_celsius(temp_ticks);
    
    if (sensor->type == SENSOR_TYPE_SHT4X && humidity) {
        uint16_t hum_ticks = sensirion_common_bytes_to_uint16_t(&buffer[2]);
        *humidity = sensirion_convert_ticks_to_percent_rh(hum_ticks);
    }
    
    return NO_ERROR;
}

int16_t sensirion_sensor_measure_with_heater(sensirion_sensor_t* sensor,
                                             sensirion_heater_power_t power,
                                             sensirion_heater_duration_t duration,
                                             int32_t* temperature,
                                             int32_t* humidity) {
    if (!sensor || !temperature || !humidity) {
        return -1;
    }
    
    if (sensor->type != SENSOR_TYPE_SHT4X) {
        return -2;  // Heater only available on SHT4X
    }
    
    uint8_t buffer[6];
    uint8_t command = heater_commands[power][duration];
    uint32_t delay = heater_delays[duration];
    
    int16_t error = sensirion_measure_ticks(
        sensor->i2c_address,
        command,
        delay,
        buffer,
        4
    );
    
    if (error) {
        return error;
    }
    
    uint16_t temp_ticks = sensirion_common_bytes_to_uint16_t(buffer);
    uint16_t hum_ticks = sensirion_common_bytes_to_uint16_t(&buffer[2]);
    
    *temperature = sensirion_convert_ticks_to_celsius(temp_ticks);
    *humidity = sensirion_convert_ticks_to_percent_rh(hum_ticks);
    
    return NO_ERROR;
}

int16_t sensirion_sensor_serial_number(sensirion_sensor_t* sensor,
                                       uint32_t* serial_number) {
    if (!sensor || !serial_number) {
        return -1;
    }
    
    return sensirion_read_serial_number(sensor->i2c_address, serial_number);
}

int16_t sensirion_sensor_soft_reset(sensirion_sensor_t* sensor) {
    if (!sensor) {
        return -1;
    }
    
    return sensirion_soft_reset(sensor->i2c_address);
}
