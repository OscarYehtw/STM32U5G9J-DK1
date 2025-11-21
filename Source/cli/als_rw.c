/*----------------------------------------------------------------------------
 *      Name:    als_rw.c
 *      Purpose: File manipulation example program
 *      Rev.:    V3.24
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2008 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>                    /* standard I/O .h-file                */
#include <stdbool.h>
#include "mxplatform.h"
#include "cli.h"
#include "tcs3410_hwdef.h"
#include "tcs3410.h"

/*----------------------------------------------------------------------------
 *        cmd_alsread
 *---------------------------------------------------------------------------*/
void cmd_alsread (char *par) {
  char *next;
  unsigned int reg = 0;
  unsigned int bytes = 0;
  uint8_t buffer[256];
  uint8_t i;
  char *pReg;
  char *pBytes;

  // Check if parameters exist
  if (par == NULL || *par == 0) {
      printf("Usage: ALSR <reg> <bytes>\n");
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

  // Perform sensor read
  memset(buffer, 0, sizeof(buffer));
    
  if (device.read)
      device.read(reg, buffer, bytes);
  else
      printf("No [ams_device_read] callback defined for this sensor.\n");

  // Output result
  printf("Read [0x%02X], %d bytes:\n", (uint8_t)reg, bytes);
  for (i = 0; i < bytes; i++) {
      printf("0x%02X ", buffer[i]);
  }
  printf("\n");
}

/*----------------------------------------------------------------------------
 *        cmd_write
 *---------------------------------------------------------------------------*/
void cmd_alswrite (char *par) {
  char *next;
  unsigned int reg = 0;
  unsigned int data = 0;
  char *pReg;
  char *pData;

  // Check parameters
  if (par == NULL || *par == 0) {
      printf("Usage: ALSW <reg> <data>\r\n");
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

  if (device.write) {
      device.write(reg, sh, data);
  } else {
      printf("No [ams_device_write] callback defined for this sensor.\n");
  }

  printf("Write [0x%02X] = 0x%02X\r\n", (uint8_t)reg, data);
}
