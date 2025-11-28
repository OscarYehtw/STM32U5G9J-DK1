/*----------------------------------------------------------------------------
 *      Name:    monitor.c
 *      Purpose: Monitor Env Sensors on LCD
 *      Rev.:    V1.0
 *----------------------------------------------------------------------------
 *      Sensirion STS4x and SHT4x sensor CLI interface
 *---------------------------------------------------------------------------*/

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include "cmsis_os2.h"
#include "cli.h"
#include "sensirion_sensor.h"

/* External sensor instances from main.c */
extern sensirion_sensor_t sht4x_sensor;  // Temperature + Humidity sensor (SHT4x, 0x44)
extern sensirion_sensor_t sts4x_sensor;  // Temperature only sensor (STS4x, 0x44)
extern osMessageQueueId_t uartRxQueueHandle;

/*----------------------------------------------------------------------------
 *        cmd_monitor  --  Display SHT/STS sensor data on LCD
 *---------------------------------------------------------------------------*/
void cmd_monitor (char *par)
{
  int32_t temperature, humidity;
  int32_t sts_temperature;
  char buf[32];
  uint8_t ch;

  printf("Press any key to exit...\n");

  while (1) {
      // Check for exit condition (non-blocking)
      if (osMessageQueueGet(uartRxQueueHandle, &ch, NULL, 0) == osOK) {
          break;
      }

      int ret_sht = sensirion_sensor_measure(&sht4x_sensor, PRECISION_HIGH, &temperature, &humidity);
      if (ret_sht == 0) {
          sprintf(buf, "SHT Temp: %0.2f C", temperature / 1000.0f);
          sprintf(buf, "SHT Humi: %0.2f %%", humidity / 1000.0f);
      } else {
          sprintf(buf, "SHT Sensor Error");
      }

      int ret_sts = sensirion_sensor_measure(&sts4x_sensor, PRECISION_HIGH, &sts_temperature, NULL);
      if (ret_sts == 0) {
          sprintf(buf, "STS Temp: %0.2f C", sts_temperature / 1000.0f);
      } else {
          sprintf(buf, "STS Sensor Error");
      }

      // Update UART console on the same line
      printf("\r"); // Move cursor to start of line
      if (ret_sht == 0) {
          printf("SHT: %0.2f C, %0.2f %% | ", temperature / 1000.0f, humidity / 1000.0f);
      } else {
          printf("SHT: Error | ");
      }

      if (ret_sts == 0) {
          printf("STS: %0.2f C", sts_temperature / 1000.0f);
      } else {
          printf("STS: Error");
      }
      printf("      "); // Print spaces to clear any trailing characters from previous longer line
      fflush(stdout);   // Force output to be sent immediately

      osDelay(1000);
  }
  
  printf("\n"); // Move to new line after exit
}
