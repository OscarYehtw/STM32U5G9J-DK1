/*----------------------------------------------------------------------------
 *      Name:    i2c_detect.c
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

int i2c_block_read(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t *data, int size)
{

  /* Timeout is set to 1S */
  while (HAL_I2C_Master_Transmit(hi2c, (uint16_t)addr << 1, (uint8_t *)&reg, 1, 1000) != HAL_OK)
  {
    /* Error_Handler() function is called when Timeout error occurs.
       When Acknowledge failure occurs (Slave don't acknowledge its address)
       Master restarts communication */

    if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
    {
      Error_Handler();
    }
	
	return HAL_ERROR;
  }
  
  while (HAL_I2C_Master_Receive(hi2c, (uint16_t)addr << 1, (uint8_t *)data, size, 10000) != HAL_OK)
  {
    /* Error_Handler() function is called when Timeout error occurs.
       When Acknowledge failure occurs (Slave don't acknowledge it's address)
       Master restarts communication */
    if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
    {
      Error_Handler();
    }

	return HAL_ERROR;
  }
  
  return HAL_OK;
}

int i2c_block_write(I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t reg, uint8_t *data, int size)
{
    int i;
    uint8_t buffer[40] = { 0 };

    buffer[0] = reg;

    
    if (size >= sizeof(buffer) - 1)
    {
        size = sizeof(buffer) - 1;
    }
    for (i = 0; i < size; ++i)
    {
        buffer[i + 1] = data[i];
    }

    /* Timeout is set to 1S */
    while (HAL_I2C_Master_Transmit(hi2c, (uint16_t)addr << 1, (uint8_t *)buffer, size + 1, 1000) != HAL_OK)
    {
      /* Error_Handler() function is called when Timeout error occurs.
         When Acknowledge failure occurs (Slave don't acknowledge its address)
         Master restarts communication */
      if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF)
      {
        Error_Handler();
      }
	
      return HAL_ERROR;
    }
	
    return HAL_OK;
}

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

  //if (bus == 3) HAL_GPIO_WritePin(TOF_LPN_GPIO_Port, TOF_LPN_Pin, GPIO_PIN_SET);

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

  //if (bus == 3) HAL_GPIO_WritePin(TOF_LPN_GPIO_Port, TOF_LPN_Pin, GPIO_PIN_RESET);
}

/*----------------------------------------------------------------------------
 *        cmd_i2cread
 *---------------------------------------------------------------------------*/
void cmd_i2cread(char *par)
{
    char *tok;
    int bus;
    uint32_t addr, reg, len = 1;
    I2C_HandleTypeDef *hi2c;
    uint8_t buf[256];

    if (par == NULL) {
        printf("Usage: I2CREAD <bus> <addr> <reg> [len]\r\n");
        return;
    }

    /* bus */
    tok = strtok(par, " ");
    if (!tok) return;
    bus = strtoul(tok, NULL, 0);

    /* addr */
    tok = strtok(NULL, " ");
    if (!tok) return;
    addr = strtoul(tok, NULL, 0);

    /* reg */
    tok = strtok(NULL, " ");
    if (!tok) return;
    reg = strtoul(tok, NULL, 0);

    /* len (optional) */
    tok = strtok(NULL, " ");
    if (tok)
        len = strtoul(tok, NULL, 0);

    if (len == 0 || len > sizeof(buf)) {
        printf("Invalid length\r\n");
        return;
    }

    hi2c = get_i2c_bus(bus);
    if (!hi2c) {
        printf("Invalid I2C bus: %d\r\n", bus);
        return;
    }

    //printf("bus: %d, addr: %lx, reg: %lx, len: %lx\r\n", bus, addr, reg, len);

    if (i2c_block_read(hi2c,
                       (uint8_t)addr,
                       (uint8_t)reg,
                       buf,
                       len) != HAL_OK)
    {
        printf("I2C read failed\r\n");
        return;
    }

    printf("I2C READ bus=%d addr=0x%02lX reg=0x%02lX : ",
           bus, addr, reg);

    for (uint32_t i = 0; i < len; i++)
        printf("%02X ", buf[i]);

    printf("\r\n");
}

/*----------------------------------------------------------------------------
 *        cmd_i2cwrite
 *---------------------------------------------------------------------------*/
void cmd_i2cwrite(char *par)
{
    char *tok;
    int bus;
    uint32_t addr, reg;
    uint8_t data[256];
    uint32_t len = 0;
    I2C_HandleTypeDef *hi2c;

    if (par == NULL) {
        printf("Usage: I2CWRITE <bus> <addr> <reg> <data...>\r\n");
        return;
    }

    /* bus */
    tok = strtok(par, " ");
    if (!tok) return;
    bus = strtoul(tok, NULL, 0);

    /* addr */
    tok = strtok(NULL, " ");
    if (!tok) return;
    addr = strtoul(tok, NULL, 0);

    /* reg */
    tok = strtok(NULL, " ");
    if (!tok) return;
    reg = strtoul(tok, NULL, 0);

    /* data bytes */
    while ((tok = strtok(NULL, " ")) != NULL) {
        if (len >= sizeof(data)) {
            printf("Too many data bytes\r\n");
            return;
        }
        data[len++] = (uint8_t)strtoul(tok, NULL, 0);
    }

    if (len == 0) {
        printf("No data specified\r\n");
        return;
    }

    hi2c = get_i2c_bus(bus);
    if (!hi2c) {
        printf("Invalid I2C bus: %d\r\n", bus);
        return;
    }

    //printf("bus: %d, addr: %lx, reg: %lx, len: %lx\r\n", bus, addr, reg, len);

    if (i2c_block_write(hi2c,
                        (uint8_t)addr,
                        (uint8_t)reg,
                        data,
                        len) != HAL_OK)
    {
        printf("I2C write failed\r\n");
        return;
    }

    printf("I2C WRITE bus=%d addr=0x%02lX reg=0x%02lX len=%lu OK\r\n",
           bus, addr, reg, len);
}

/*----------------------------------------------------------------------------
 *        cmd_ioexpr
 *---------------------------------------------------------------------------*/
void cmd_ioexpr (char *par) {
  char *next;
  unsigned int reg = 0;
  unsigned int bytes = 0;
  uint8_t buffer[256];
  uint8_t i;
  char *pReg;
  char *pBytes;
  uint8_t addr = 0x20;
  I2C_HandleTypeDef *hi2c = &hbus_i2c6;

  // Check if parameters exist
  if (par == NULL || *par == 0) {
      printf("Usage: IOEXR <reg> <bytes>\n");
      return;
  }

  // Get the first parameter (register)
  pReg = get_entry(par, &next);
  if (pReg == NULL) {
      printf("Invalid register address.\n");
      return;
  }

  // Convert register (hexadecimal)
  if (sscanf(pReg, "%x", &reg) != 1) {
      printf("Invalid register format.\n");
      return;
  }

  // Get the second parameter (bytes)
  pBytes = get_entry(next, &next);
  if (pBytes == NULL) {
      printf("Missing bytes count.\n");
      return;
  }

  // Convert byte count
  if (sscanf(pBytes, "%x", &bytes) != 1 || bytes == 0 || bytes > sizeof(buffer)) {
      printf("Invalid byte count (1~%d allowed)\n", (int)sizeof(buffer));
      return;
  }

  if (i2c_block_read(hi2c, addr, reg, buffer, bytes) != HAL_OK)
  {
      printf("I2C read failed\r\n");
      return;
  }

  // Output result
  printf("Read [0x%02X], %d bytes:\n", (uint8_t)reg, bytes);
  for (i = 0; i < bytes; i++) {
      printf("0x%02X ", buffer[i]);
  }
  printf("\n");
}

/*----------------------------------------------------------------------------
 *        cmd_ioexpw
 *---------------------------------------------------------------------------*/
void cmd_ioexpw (char *par) {
  char *next;
  unsigned int reg = 0;
  unsigned int data = 0;
  char *pReg;
  char *pData;
  uint8_t tx;
  uint8_t addr = 0x20;
  I2C_HandleTypeDef *hi2c = &hbus_i2c6;

  // Check parameters
  if (par == NULL || *par == 0) {
      printf("Usage: IOEXW <reg> <data>\r\n");
      return;
  }

  // Get the first parameter (register)
  pReg = get_entry(par, &next);
  if (pReg == NULL) {
      printf("Invalid register address.\r\n");
      return;
  }

  // Convert register (hexadecimal)
  if (sscanf(pReg, "%x", &reg) != 1) {
      printf("Invalid register format.\r\n");
      return;
  }

  // Get the second parameter (data)
  pData = get_entry(next, &next);
  if (pData == NULL) {
      printf("Missing data value.\r\n");
      return;
  }

  // Convert data (hexadecimal)
  if (sscanf(pData, "%x", &data) != 1) {
      printf("Invalid data format.\r\n");
      return;
  }

  tx = (uint8_t)data;

  if (i2c_block_write(hi2c, addr, (uint8_t)reg, &tx, 1) != HAL_OK)
  {
      printf("I2C write failed\r\n");
      return;
  }

  printf("Write [0x%02X] = 0x%02X\r\n", (uint8_t)reg, tx);
}
