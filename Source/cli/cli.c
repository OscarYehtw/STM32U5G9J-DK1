/*----------------------------------------------------------------------------
 *      Name:    CLI.C
 *      Purpose: File manipulation example program
 *      Rev.:    V3.24
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2008 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>                    /* standard I/O .h-file                */
#include <string.h>                   /* string and memory functions         */
#include <ctype.h>                    /* character functions                 */
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_freertos.h"
#include "main.h"
#include "cli.h"
#include "ams_device.h"
#include "tcs3410_hwdef.h"
#include "tcs3410.h"

extern TIM_HandleTypeDef htim8;

const char Cli_Help[] = 
   "+ COMMAND ------------------+ FUNCTION ---------------------------------+\n"
   "| ALSR <reg> <bytes>        | Read <bytes> from sensor register <reg>   |\n"
   "| ALSW <reg> <data >        | Write <data> to sensor register <reg>     |\n"
   "| TMR                       | Show TMR-ADC values                       |\n"
   "| FILL <rgb888>             | Fill screen with rgb color                |\n"
   "| BL   <brightness>         | set backlight to brightness [0-100%%]      |\n"
   "| HELP  or  ?               | displays this help                        |\n"
   "+---------------------------+-------------------------------------------+\n";

const SCMD cmd[] = {
   "ALSR",   cmd_alsread,
   "ALSW",   cmd_alswrite,
   "TMR",    cmd_tmradc,
   "FILL",   cmd_fill,
   "BL",     cmd_backlight,
   "HELP",   cmd_help,
   "?",      cmd_help};

#define CMD_COUNT   (sizeof (cmd) / sizeof (cmd[0]))

char in_line[160];

/*----------------------------------------------------------------------------
 *        cmd_alsread
 *---------------------------------------------------------------------------*/
void cmd_alsread (char *par) {
  char *next;
  uint32_t reg = 0;
  uint32_t bytes = 0;
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
  uint32_t reg = 0;
  uint32_t data = 0;
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

/*----------------------------------------------------------------------------
 *        cmd_tmradc  --  Display ADC4 conversion results
 *---------------------------------------------------------------------------*/
void cmd_tmradc (char *par)
{
  (void)par;
  const float VREF     = 3.3f;
  const float ADC_MAX  = 4095.0f;
  char buf[32];
  uint32_t adc1, adc2;
  float v1, v2;

  printf("=== TMR-ADC monitor mode ===\n");
  printf("Press [ESC] to exit.\n\n");

  while (1)
  {
      taskENTER_CRITICAL();
      adc1 = aADCxConvertedData[0];
      adc2 = aADCxConvertedData[1];
      taskEXIT_CRITICAL();

      v1 = (adc1 * VREF) / ADC_MAX;
      v2 = (adc2 * VREF) / ADC_MAX;
      printf("ADC1=%lu (%.3f V), ADC2=%lu (%.3f V)\n", (unsigned long)adc1, v1, (unsigned long)adc1, v1);

      int ch = (int) READ_REG(huart1.Instance->RDR);
      if (ch == ESC)  // ESC key
      {
          printf("\nExit TMR-ADC monitor.\n");
          break;
      }
      osDelay(500);
  }
}

/*----------------------------------------------------------------------------
 *        cmd_fill  --  Fill screen with rgb color
 *---------------------------------------------------------------------------*/
void cmd_fill (char *par)
{
  char *pRGB;
  uint32_t rgb888;

  if (par == NULL || *par == 0) {
      printf("Usage: FILL <rgb888>\n");
      printf("Example: FILL FF0000 (red)\n");
      return;
  }

  pRGB = get_entry(par, &par);
  if (pRGB == NULL) {
      printf("Invalid parameter.\n");
      return;
  }

  if (sscanf(pRGB, "%x", &rgb888) != 1) {
      printf("Invalid rgb888 format.\n");
      return;
  }

  UTIL_LCD_FillRect(0, 0, LCD_WIDTH, LCD_HEIGHT, rgb888);
}

/*----------------------------------------------------------------------------
 *        cmd_backlight  --  set backlight to brightness
 *---------------------------------------------------------------------------*/
void cmd_backlight (char *par)
{
  char *pVal;
  int percent;

  // No argument ? show usage
  if (par == NULL || *par == 0) {
      printf("Usage: BL <brightness>\n");
      printf("Brightness range: 0 ~ 100\n");
      return;
  }

  // Parse CLI argument
  pVal = get_entry(par, &par);
  if (pVal == NULL) {
      printf("Invalid parameter.\n");
      return;
  }

  // Convert to integer
  if (sscanf(pVal, "%d", &percent) != 1) {
      printf("Invalid brightness value.\n");
      return;
  }

  // Range clip
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  // Convert percentage to duty (Period = 139)
  uint32_t duty = (139 * percent) / 100;

  // Apply PWM duty
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty);

  printf("Backlight = %d%% (duty=%lu)\n", percent, (unsigned long)duty);
}

/*----------------------------------------------------------------------------
 *        Display Command Syntax help
 *---------------------------------------------------------------------------*/
void cmd_help (char *par) {
   printf (Cli_Help);
}

/*----------------------------------------------------------------------------
 *      Line Editor
 *---------------------------------------------------------------------------*/
bool getline (char *lp, uint32_t n) {
   uint32_t cnt = 0;
   char c;

   do {
      c = getkey ();
      switch (c) {
         case CNTLQ:                       /* ignore Control S/Q             */
         case CNTLS:
            break;;
         case BACKSPACE:
         case DEL:
            if (cnt == 0) {
               break;
            }
            cnt--;                         /* decrement count                */
            lp--;                          /* and line pointer               */
            putchar (0x08);                /* echo backspace                 */
            putchar (' ');
            putchar (0x08);
            break;
         case ESC:
            *lp = 0;                       /* ESC - stop editing line        */
            return (false);
         case CR:                          /* CR - done, stop editing line   */
            *lp = c;
            lp++;                          /* increment line pointer         */
            cnt++;                         /* and count                      */
            c = LF;
         default:
            putchar (*lp = c);             /* echo and store character       */
            lp++;                          /* increment line pointer         */
            cnt++;                         /* and count                      */
            break;
      }
   } while (cnt < n - 2  &&  c != LF);     /* check limit and CR             */
   *lp = 0;                                /* mark end of string             */
   return (true);
}

/*----------------------------------------------------------------------------
 *        Process input string for long or short name entry
 *---------------------------------------------------------------------------*/
char *get_entry (char *cp, char **pNext) {
   char *sp, lfn = 0, sep_ch = ' ';

   if (cp == NULL) {                          /* skip NULL pointers          */
      *pNext = cp;
      return (cp);
   }

   for ( ; *cp == ' ' || *cp == '\"'; cp++) { /* skip blanks and starting  " */
      if (*cp == '\"') { sep_ch = '\"'; lfn = 1; }
      *cp = 0;
   }
 
   for (sp = cp; *sp != CR && *sp != LF; sp++) {
      if ( lfn && *sp == '\"') break;
      if (!lfn && *sp == ' ' ) break;
   }

   for ( ; *sp == sep_ch || *sp == CR || *sp == LF; sp++) {
      *sp = 0;
      if ( lfn && *sp == sep_ch) { sp ++; break; }
   }

   *pNext = (*sp) ? sp : NULL;                /* next entry                  */
   return (cp);
}

/*----------------------------------------------------------------------------
 *        DispatchCmd: 
 *---------------------------------------------------------------------------*/
void DispatchCmd (void) {
	char *sp,*cp,*next;
	uint32_t i;

   /* display prompt */
   printf ("\nCmd> ");

   /* get command line input */
	if (getline (in_line, sizeof (in_line)) == false) {
		return;
	}

	sp = get_entry (&in_line[0], &next);
	if (*sp == 0) {
		return;
    }
	
	for (cp = sp; *cp && *cp != ' '; cp++) {
      /* command to upper-case */
		*cp = toupper (*cp);
	}
    
	for (i = 0; i < CMD_COUNT; i++) {
   
		if (strcmp (sp, (const char *)&cmd[i].val)) {
			continue;
		}

		/* execute command function */
		cmd[i].func (next);
		break;                           
	}
	
	if (i == CMD_COUNT) {
		printf ("\nCommand error\n");
	}
	return;
}
