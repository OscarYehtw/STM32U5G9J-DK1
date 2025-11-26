/*----------------------------------------------------------------------------
 *      Name:    als_rw.c
 *      Purpose: File manipulation example program
 *      Rev.:    V3.24
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2008 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>                    /* standard I/O .h-file                */
#include <stdlib.h>
#include <stdbool.h>
#include "mxplatform.h"
#include "cli.h"

/* Return pointer to I2C handle by bus index */
static I2C_HandleTypeDef *get_i2c_bus(int bus)
{
  switch (bus) {
      case 1:
             if (HAL_I2C_GetState(&hbus_i2c1) == HAL_I2C_STATE_RESET)
               BSP_I2C1_Init();
             return &hbus_i2c1;
      case 2:
             if (HAL_I2C_GetState(&hbus_i2c2) == HAL_I2C_STATE_RESET)
               BSP_I2C2_Init();
             return &hbus_i2c2;
      case 3:
             if (HAL_I2C_GetState(&hbus_i2c3) == HAL_I2C_STATE_RESET)
               BSP_I2C3_Init();
             return &hbus_i2c3;
      case 4:
             if (HAL_I2C_GetState(&hbus_i2c4) == HAL_I2C_STATE_RESET)
               BSP_I2C4_Init();
    	     return &hbus_i2c4;
      case 5:
             if (HAL_I2C_GetState(&hbus_i2c5) == HAL_I2C_STATE_RESET)
               BSP_I2C5_Init();
    	     return &hbus_i2c5;
      case 6:
             if (HAL_I2C_GetState(&hbus_i2c6) == HAL_I2C_STATE_RESET)
               BSP_I2C6_Init();
    	     return &hbus_i2c6;
      default:
             return NULL;
  }
}

/*----------------------------------------------------------------------------
 *        cmd_i2cdetect
 *---------------------------------------------------------------------------*/
void cmd_i2cdetect (char *par) {
  int bus = 1;                 // default bus 1
  I2C_HandleTypeDef *hi2c;

  /* parse parameter */
  if (par != NULL && *par != 0) {
      bus = strtoul(par, NULL, 0);
  }

  if (bus == 3) HAL_GPIO_WritePin(TOF_LPN_GPIO_Port, TOF_LPN_Pin, GPIO_PIN_SET);

  /* get I2C handle */
  hi2c = get_i2c_bus(bus);
  if (hi2c == NULL) {
      printf("Invalid I2C bus: %d\r\n", bus);
      return;
  }

  printf("Scanning I2C bus...\r\n");
  printf("    00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");

  for (uint8_t row = 0; row < 8; row++) {
      printf("%02X: ", row << 4);

      for (uint8_t col = 0; col < 16; col++) {
          uint8_t addr = (row << 4) | col;

          if (addr < 0x03 || addr > 0x77) {
              printf("   ");
              continue;
          }

          if (HAL_I2C_IsDeviceReady(hi2c, (addr << 1), 2, 10) == HAL_OK) {
              printf("%02X ", addr);
          } else {
              printf("-- ");
          }
      }
      printf("\r\n");
  }

  if (bus == 3) HAL_GPIO_WritePin(TOF_LPN_GPIO_Port, TOF_LPN_Pin, GPIO_PIN_RESET);
}
