/*----------------------------------------------------------------------------
 *      Name:    lcdctrl.c
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
#include "lm3697.h"

extern TIM_HandleTypeDef htim8;

/*----------------------------------------------------------------------------
 *        cmd_fill  --  Fill screen with rgb color
 *---------------------------------------------------------------------------*/
void cmd_fill (char *par)
{
  char *pRGB;
  unsigned int rgb888;

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
#if defined(BRIGHTNESS_USE_LM3597) && (BRIGHTNESS_USE_LM3597 == 1)
  int brightness;
#else
  int percent;
#endif

  // No argument ? show usage
  if (par == NULL || *par == 0) {
      printf("Usage: BL <brightness>\n");
#if defined(BRIGHTNESS_USE_LM3597) && (BRIGHTNESS_USE_LM3597 == 1)
      printf("Brightness range: 0 ~ 255\n");
#else
      printf("Brightness range: 0 ~ 100\n");
#endif
      return;
  }

  // Parse CLI argument
  pVal = get_entry(par, &par);
  if (pVal == NULL) {
      printf("Invalid parameter.\n");
      return;
  }

  // Convert to integer
#if defined(BRIGHTNESS_USE_LM3597) && (BRIGHTNESS_USE_LM3597 == 1)
  if (sscanf(pVal, "%d", &brightness) != 1) {
#else
  if (sscanf(pVal, "%d", &percent) != 1) {
#endif
	  printf("Invalid brightness value.\n");
      return;
  }

#if defined(BRIGHTNESS_USE_LM3597) && (BRIGHTNESS_USE_LM3597 == 1)
  printf("Using LM3697 to control backlight brightness.\n");
  BL_Driver_SetBrightness_8bit(brightness);
  return;
#else
  // Range clip
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;

  // Convert percentage to duty (Period = 139)
  uint32_t duty = (139 * percent) / 100;

  // Apply PWM duty
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, duty);

  printf("Backlight = %d%% (duty=%lu)\n", percent, (unsigned long)duty);
#endif
}
