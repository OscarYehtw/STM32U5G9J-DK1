/*----------------------------------------------------------------------------
 *      Name:    tmr_adc.c
 *      Purpose: File manipulation example program
 *      Rev.:    V3.24
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2008 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stdio.h>                    /* standard I/O .h-file                */
#include <stdbool.h>
#include <math.h>
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cli.h"

#define PI 3.14159265358979f

/*----------------------------------------------------------------------------
 *        cmd_dialstart  --  Display ADC4 conversion results
 *---------------------------------------------------------------------------*/
void cmd_dialstart (char *par)
{
  (void)par;
  const float VREF     = 3.3f;
  const float ADC_MAX  = 4095.0f;
  const uint32_t SAMPLE_DELAY_MS = 10; /* Sampling interval (ms) */
  const uint32_t STABLE_SAMPLES  = 6;  /* Additional stable samples after detecting an extrema sequence; adjustable */
  const int32_t EDGE_DEADBAND    = 4;  /* Trend-change debounce threshold (ADC counts) */

  uint32_t adc1 = 0, adc2 = 0;
  uint32_t min1 = 0xFFFFFFFFUL, max1 = 0;
  uint32_t min2 = 0xFFFFFFFFUL, max2 = 0;

  uint32_t prev_adc1, prev_adc2;
  int8_t trend1 = 0, trend2 = 0; /* 1=rising, -1=falling, 0=unknown */
  uint32_t stable_count1 = 0, stable_count2 = 0;
  
  bool saw_seq1 = false, saw_seq2 = false; /* Whether a max->min or min->max sequence has been observed */
  uint8_t last_extreme1 = 0; /* 1=min, 2=max */
  uint8_t last_extreme2 = 0;

  float v1_min, v1_max, v1_mid, v1_amp, v2_min, v2_max, v2_mid, v2_amp;
  float v1_cal, v2_cal, angle_deg;

  printf("=== TMR-ADC Automatic Sequence Calibration Monitor ===\n");
  printf("Goal: Each channel must observe a full 'max-to-min'\n");
  printf("      or 'min-to-max' sequence to complete calibration.\n\n");
  printf("If the ADC value does not change, sampling will continue\n");
  printf("until the sequence is completed.\n");
  printf("Press [ESC] to abort.\n\n");

  /* Take the first sample as prev */
  taskENTER_CRITICAL();
  prev_adc1 = aADCxConvertedData[0];
  prev_adc2 = aADCxConvertedData[1];
  taskEXIT_CRITICAL();

  min1 = max1 = prev_adc1;
  min2 = max2 = prev_adc2;

  /* Continue sampling until both channels have each completed one extrema sequence */
  while (!(saw_seq1 && saw_seq2))
  {
      taskENTER_CRITICAL();
      adc1 = aADCxConvertedData[0];
      adc2 = aADCxConvertedData[1];
      taskEXIT_CRITICAL();

      /* Update global min/max */
      if (adc1 < min1) min1 = adc1;
      if (adc1 > max1) max1 = adc1;
      if (adc2 < min2) min2 = adc2;
      if (adc2 > max2) max2 = adc2;

      /* --- Channel 1: Trend detection and extrema identification --- */
      if (!saw_seq1) {
        if ((int32_t)adc1 > (int32_t)prev_adc1 + EDGE_DEADBAND) {
          /* rising */
          if (trend1 == -1) {
            /* falling -> rising : trough (min) detected */
            uint32_t trough = prev_adc1;
            if (trough < min1) min1 = trough;
            if (last_extreme1 == 2) {
              /* previously saw max -> now min => max->min sequence completed */
              saw_seq1 = true;
              stable_count1 = 0;
            } else {
              last_extreme1 = 1; /* mark min */
            }
          }
          trend1 = 1;
        } else if ((int32_t)adc1 < (int32_t)prev_adc1 - EDGE_DEADBAND) {
          /* falling */
          if (trend1 == 1) {
            /* rising -> falling : peak (max) detected */
            uint32_t peak = prev_adc1;
            if (peak > max1) max1 = peak;
            if (last_extreme1 == 1) {
              /* previously saw min -> now max => min->max sequence completed */
              saw_seq1 = true;
              stable_count1 = 0;
            } else {
              last_extreme1 = 2; /* mark max */
            }
          }
          trend1 = -1;
        }
      } else {
        /* If sequence already observed, collect extra stable samples before finishing (to avoid stopping too early) */
        if ((max1 > min1) && stable_count1 < STABLE_SAMPLES) {
          stable_count1++;
        }
      }

      /* --- Channel 2: same as above --- */
      if (!saw_seq2) {
        if ((int32_t)adc2 > (int32_t)prev_adc2 + EDGE_DEADBAND) {
          if (trend2 == -1) {
            uint32_t trough = prev_adc2;
            if (trough < min2) min2 = trough;
            if (last_extreme2 == 2) {
              saw_seq2 = true;
              stable_count2 = 0;
            } else {
              last_extreme2 = 1;
            }
          }
          trend2 = 1;
        } else if ((int32_t)adc2 < (int32_t)prev_adc2 - EDGE_DEADBAND) {
          if (trend2 == 1) {
            uint32_t peak = prev_adc2;
            if (peak > max2) max2 = peak;
            if (last_extreme2 == 1) {
              saw_seq2 = true;
              stable_count2 = 0;
            } else {
              last_extreme2 = 2;
            }
          }
          trend2 = -1;
        }
      } else {
        if ((max2 > min2) && stable_count2 < STABLE_SAMPLES) {
          stable_count2++;
        }
      }

      /* Check ESC abort */
      int ch = (int) READ_REG(huart1.Instance->RDR);
      if (ch == ESC) {
          printf("\nCalibration aborted.\n");
          return;
      }

      /* Update prev */
      prev_adc1 = adc1;
      prev_adc2 = adc2;

      osDelay(SAMPLE_DELAY_MS);
  }

  v1_min = (float)min1 * VREF / ADC_MAX;   // volts
  v1_max = (float)max1 * VREF / ADC_MAX;
  v1_mid = (v1_max + v1_min) / 2.0f;
  v1_amp = v1_max - v1_min;

  v2_min = (float)min2 * VREF / ADC_MAX;
  v2_max = (float)max2 * VREF / ADC_MAX;
  v2_mid = (v2_max + v2_min) / 2.0f;
  v2_amp = v2_max - v2_min;

  printf("Calibration complete: \n");
  printf("V1 Min=%.3f V, V1 Max=%.3f V, V1 Center=%.3f V, V1 Amp=%.3f V\n",
         v1_min, v1_max, v1_mid, v1_amp);
  printf("V2 Min=%.3f V, V2 Max=%.3f V, V2 Center=%.3f V, V2 Amp=%.3f V\n",
         v2_min, v2_max, v2_mid, v2_amp);
  printf("Entering monitor (shows Angle Dgree).\n");
  printf("Press [ESC] to exit monitoring.\n\n");

  while (1)
  {
      taskENTER_CRITICAL();
      adc1 = aADCxConvertedData[0];
      adc2 = aADCxConvertedData[1];
      taskEXIT_CRITICAL();

      v1_cal = ((float)adc1 * VREF / ADC_MAX - v1_mid) / v1_amp;  // -1..1
      v2_cal = ((float)adc2 * VREF / ADC_MAX - v2_mid) / v2_amp;  // -1..1
      angle_deg = atan2f(v2_cal, v1_cal) * 180.0f / PI;

      if (angle_deg < 0) angle_deg += 360.0f;

      printf("ADC1=%04ld (%.2f V), ADC2=%04ld (%.2f V), TMR Angle:%.3f\n", \
              (long unsigned int)adc1, (adc1 * VREF / ADC_MAX), \
              (long unsigned int)adc2, (adc2 * VREF / ADC_MAX), angle_deg);

      int ch = (int) READ_REG(huart1.Instance->RDR);
      if (ch == ESC)  // ESC key
      {
          printf("\nExit TMR-ADC monitor.\n");
          break;
      }
      osDelay(20);
  }

}
