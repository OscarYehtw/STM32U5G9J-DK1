/*----------------------------------------------------------------------------
 *      Name:    tmp_hum.c
 *      Purpose: Temperature and Humidity sensor control
 *      Rev.:    V1.0
 *----------------------------------------------------------------------------
 *      Sensirion STS4x and SHT4x sensor CLI interface
 *---------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include "cli.h"
#include "sensirion_sensor.h"

/* External sensor instances from main.c */
extern sensirion_sensor_t sht4x_sensor;  // Temperature + Humidity sensor (SHT4x, 0x44)
extern sensirion_sensor_t sts4x_sensor;  // Temperature only sensor (STS4x, 0x44)

/*----------------------------------------------------------------------------
 *        cmd_temp  --  Read temperature from sensor
 *---------------------------------------------------------------------------*/
void cmd_temp (char *par) {
    char *pSensor;
    int32_t temp = 0;
    int ret;
    
    // Check if parameter exists
    if (par == NULL || *par == 0) {
        printf("Usage: TEMP <sht|sts>\n");
        return;
    }
    
    // Get sensor type parameter
    pSensor = get_entry(par, &par);
    if (pSensor == NULL) {
        printf("Invalid parameter.\n");
        return;
    }
    
    // Convert to uppercase for comparison
    for (char *p = pSensor; *p; p++) {
        *p = toupper(*p);
    }
    
    // Read from specified sensor
    if (strcmp(pSensor, "SHT") == 0) {
        ret = sensirion_sensor_measure(&sht4x_sensor, PRECISION_HIGH, &temp, NULL);
        if (ret == 0) {
            printf("temp: %ld.%03ld deg C (SHT4x)\n", (long)(temp / 1000), (long)(temp % 1000));
        } else {
            printf("SHT4x Read Error\n");
        }
    } else if (strcmp(pSensor, "STS") == 0) {
        ret = sensirion_sensor_measure(&sts4x_sensor, PRECISION_HIGH, &temp, NULL);
        if (ret == 0) {
            printf("temp: %ld.%03ld deg C (STS4x)\n", (long)(temp / 1000), (long)(temp % 1000));
        } else {
            printf("STS4x Read Error\n");
        }
    } else {
        printf("Invalid sensor type. Use 'sht' or 'sts'\n");
    }
}

/*----------------------------------------------------------------------------
 *        cmd_hum  --  Read humidity from sensor
 *---------------------------------------------------------------------------*/
void cmd_hum (char *par) {
    int32_t temp = 0, hum = 0;
    int ret;
    
    // Read from SHT4x sensor (humidity)
    ret = sensirion_sensor_measure(&sht4x_sensor, PRECISION_HIGH, &temp, &hum);
    
    // Output result
    if (ret == 0) {
        printf("hum: %ld.%03ld %%RH\n", hum / 1000, hum % 1000);
    } else {
        printf("Humidity Read Error\n");
    }
}

/*----------------------------------------------------------------------------
 *        cmd_heat  --  Activate heater and measure
 *---------------------------------------------------------------------------*/
void cmd_heat (char *par) {
    char *pPower, *pDuration;
    sensirion_heater_power_t power = HEATER_POWER_HIGHEST;
    sensirion_heater_duration_t duration = HEATER_DURATION_LONG;
    int32_t temp = 0, hum = 0;
    int ret;

    // Parse Power
    pPower = get_entry(par, &par);
    if (pPower == NULL) {
        printf("Usage: HEAT <HIGH|MED|LOW> <LONG|SHORT>\n");
        return;
    }
    
    for (char *p = pPower; *p; p++) *p = toupper(*p);
    
    if (strcmp(pPower, "HIGH") == 0) power = HEATER_POWER_HIGHEST;
    else if (strcmp(pPower, "MED") == 0) power = HEATER_POWER_MEDIUM;
    else if (strcmp(pPower, "LOW") == 0) power = HEATER_POWER_LOWEST;
    else {
        printf("Invalid power. Use HIGH, MED, or LOW\n");
        return;
    }

    // Parse Duration
    pDuration = get_entry(par, &par);
    if (pDuration == NULL) {
        printf("Usage: HEAT <HIGH|MED|LOW> <LONG|SHORT>\n");
        return;
    }

    for (char *p = pDuration; *p; p++) *p = toupper(*p);

    if (strcmp(pDuration, "LONG") == 0) duration = HEATER_DURATION_LONG;
    else if (strcmp(pDuration, "SHORT") == 0) duration = HEATER_DURATION_SHORT;
    else {
        printf("Invalid duration. Use LONG or SHORT\n");
        return;
    }

    printf("Activating heater (Power: %s, Duration: %s)...\n", pPower, pDuration);

    ret = sensirion_sensor_measure_with_heater(&sht4x_sensor, power, duration, &temp, &hum);

    if (ret == 0) {
        printf("Result - Temp: %ld.%03ld C, Hum: %ld.%03ld %%RH\n", 
               (long)(temp / 1000), (long)(temp % 1000), (long)(hum / 1000), (long)(hum % 1000));
    } else {
        printf("Heater Measurement Error: %d\n", ret);
    }
}
