/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
 * USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
 * EXCLUDED.                                                                 *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
  * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
  * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */

#include <stdio.h>                    /* standard I/O .h-file                */
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "ams_device.h"
#include "tcs3410_hwdef.h"
#include "ams_cli.h"
#include "tcs3410.h"
#include "tcs3410_als.h"

static const char *photodiode_connect[][3] =
{
    {"CLEAR", "GREEN", "RED"},
    {"CLEAR", "BLUE", "WIDEBAND"},
    {"CLEAR", "BLUE", "RED"},
};

static const double cct_coeffs[][N_MAX] =
{
                    /* N_LO    N_MED   N_HI */
    [COEFA]        = { 5239,   8096,   215},
    [CTOFFSET]     = { 1747,   1518,   2309},
};

#define IR_COMP_RATIO_LO       (0.078)
#define IR_COMP_RATIO_HI       (0.131)

static const double lux_coeffs[][3] =
{
                     /* N_LO    N_MED   N_HI */
   [RED_COEFF]    = { 0.861, -1.37,   0.461},
   [GREEN_COEFF]  = { 2.004, -1.197, -0.186},
   [BLUE_COEFF]   = { 0.132, -0.659,  0.778},
   [WB_COEFF]     = { 0.671,  0.015, -0.051},
   [CLEAR_COEFF]  = {-1.895,  1.808,  0.199},
   [DGF]          = { 5.0,    5.0,    5.0},

};

#define GC_IR          (1.0)
// TODO: NOM
#define GC_ATIME_MS    (200.0)

#define MAGIC_MAX      (100.0)
#define MAGIC_ALT      (0.5)

/*
 *  ATIME is no longer directly used on Betz.  It can be derived
 *  from Sample_Time and ALS_Nr_Samples
 */
static double calculate_atime_ms(void)
{
    uint32_t atime_ms;
    uint16_t als_nr_samples = sensor_get_als_nr_samples();
    uint16_t sample_time    = sensor_get_sample_time();

    //    atime = (pdevice_info->sample_time) * (pdevice_info->als_nr_samples) * MOD_CLOCK_STEP_MS;
    atime_ms = (double)(als_nr_samples * sample_time) * MOD_CLOCK_STEP_MS;
    return(atime_ms); 
}

static void show_als_data(ams_als_info_t als_data[])
{
    int step;

    printf("\n");
    printf("ALS data:\n");
    for (step = 0; step < ALS_NUM_STEPS; step++)
    {
        printf("Step %d:  mod_0(%s): count:%5ld, gain:0x%02X[%dx] \n",
			 step, photodiode_connect[step][0], als_data[step].mod_counts[0], als_data[step].gains[MODULATOR_0], (int)als_data[step].mod_gains[MODULATOR_0]);
        printf("Step %d:  mod_1(%s): count:%5ld, gain:0x%02X[%dx] \n",
                         step, photodiode_connect[step][1], als_data[step].mod_counts[1], als_data[step].gains[MODULATOR_1], (int)als_data[step].mod_gains[MODULATOR_1]);
        printf("Step %d:  mod_2(%s): count:%5ld, gain:0x%02X[%dx] \n",
			 step, photodiode_connect[step][2], als_data[step].mod_counts[2], als_data[step].gains[MODULATOR_2], (int)als_data[step].mod_gains[MODULATOR_2]);
        printf("\n");
    }

    return;
}

static bool check_for_saturation(ams_als_info_t als_data[])
{
    bool ret = false;
    uint16_t step;
    uint8_t ch0_sat, ch1_sat, ch2_sat, seq_step;

    for (step = 0; step < ALS_NUM_STEPS; step++)
    {
        seq_step = (als_data[step].status[ALS_STATUS_REG_INDEX] & ALS_STATUS_MEAS_SEQ_STEP_MASK) >> ALS_STATUS_MEAS_SEQ_STEP_SHIFT;
        ch0_sat  = (als_data[step].status[ALS_STATUS_REG_INDEX] & ALS_STATUS_ANA_SAT_MOD0_MASK) >> ALS_STATUS_ANA_SAT_MOD0_SHIFT;
        ch1_sat  = (als_data[step].status[ALS_STATUS_REG_INDEX] & ALS_STATUS_ANA_SAT_MOD1_MASK) >> ALS_STATUS_ANA_SAT_MOD1_SHIFT;
        ch2_sat  = (als_data[step].status[ALS_STATUS_REG_INDEX] & ALS_STATUS_ANA_SAT_MOD2_MASK) >> ALS_STATUS_ANA_SAT_MOD0_SHIFT;

	if (ch0_sat || ch1_sat|| ch2_sat)
        {
            printf("Saturation occured during ALS Sequence Step: %d (%d, %d, %d)", seq_step, ch0_sat, ch1_sat, ch2_sat);
            ret = true;
            break;
        }
    }
    return(ret);
}

static void als_calc_lux(ams_device_status_t *stat, uint8_t log)
{
    ams_als_info_t als_data[ALS_NUM_STEPS];
    uint16_t step, mods, status_regs, idx;
    double average_counts[NUM_PD], ir_comp, ir_comp_ratio, atime_correction, lux, cct;
    uint8_t n = 0;
    double atime;
    double r_prime;
    double b_prime;

#if defined(DEBUG_SHOW_ALS_FIFO)
    NRF_LOG_RAW_INFO("In als_calc_lux fifo data \n");
    for (idx = 1; idx <= ALS_FIFO_DATA_LENGTH; idx++)
    {
      printf("%2x ", stat->als.als_fifo_data[idx-1]);
      if ((idx % 9) == 0)
      {
         printf("\n");
      }
    }
#endif

    cct = 0;
    lux = 0.0;
    idx = 0;
    /* Convert Data Format */
    for (step = 0; step < ALS_NUM_STEPS; step++)
    {
        for (mods = 0; mods < NUM_MODULATORS; mods++)
        {
            als_data[step].mod_counts[mods] = (((stat->als.als_fifo_data[idx+2] & 0xFF) << 16) |
                                         ((stat->als.als_fifo_data[idx+1] & 0xFF) << 8) |
                                         ((stat->als.als_fifo_data[idx] & 0xFF) << 0)
                                         );
            idx += 3;
        }

        /* als_data[step].status[0] contains the saturation information */
        for (status_regs = 0; status_regs < ALS_NUM_STATUS_REGS; status_regs++)
        {
            als_data[step].status[status_regs] = stat->als.als_fifo_data[idx++];

        }

        als_data[step].gains[MODULATOR_0] = ((als_data[step].status[ALS_STATUS2_REG_INDEX]& 0x0F) >> 0);
        als_data[step].gains[MODULATOR_1] = ((als_data[step].status[ALS_STATUS2_REG_INDEX]& 0xF0) >> 4);
        als_data[step].gains[MODULATOR_2] = ((als_data[step].status[ALS_STATUS3_REG_INDEX]& 0x0F) >> 0);

        als_data[step].mod_gains[MODULATOR_0] = (1 << (als_data[step].gains[MODULATOR_0] - 1));
        als_data[step].mod_gains[MODULATOR_1] = (1 << (als_data[step].gains[MODULATOR_1] - 1));
        als_data[step].mod_gains[MODULATOR_2] = (1 << (als_data[step].gains[MODULATOR_2] - 1));
    }

    if ( log == 1 )
    {
        show_als_data(&als_data[0]);
    }

    if (check_for_saturation(&als_data[0]))
    {
        printf("Terminating sequence due to analog saturation \n");
        stat->als.lux = 0;
        return;
    }

    /* Start Normalizating the Channel Data */
    /* Calculate Gain ratios where step 0, mod 0 (Clear) is the reference */
    /* This logic only works because the reference is step 0, mod 0 */
    /* If you change the reference, this logic needs to change */
    for (step = 0; step < ALS_NUM_STEPS; step++)
    {
        for (mods = 0; mods < NUM_MODULATORS; mods++)
        {
            /* The reference gain remains constant and the counts are not adjusted*/
            if ((step == STEP_0) && (mods == MODULATOR_0))
            {
                als_data[STEP_0].mod_normalized_gains[MODULATOR_0]  = als_data[STEP_0].mod_gains[MODULATOR_0];
                als_data[STEP_0].mod_normalized_counts[MODULATOR_0] = (double)als_data[STEP_0].mod_counts[MODULATOR_0];
            }
            else
            {
                /* Normalize all other channel data to step 0, mod 0 */
                als_data[step].mod_normalized_gains[mods]  = als_data[step].mod_gains[mods]/als_data[STEP_0].mod_gains[MODULATOR_0];

                /* Normalize the [step,mod] count to that of its normalized gain */
                als_data[step].mod_normalized_counts[mods] = (double)als_data[step].mod_counts[mods]/als_data[step].mod_normalized_gains[mods];
            }
        }
    }

    /* Calculate matching factor for the clear channel that appears in all 3 ALS steps */
    als_data[0].matching_factors = 1;
    als_data[1].matching_factors = als_data[1].mod_normalized_counts[0] / als_data[0].mod_normalized_counts[0];
    als_data[2].matching_factors = als_data[2].mod_normalized_counts[0] / als_data[0].mod_normalized_counts[0];

    /* The atime can be corrected by using the flicker calculation during this round and adjust the atime */
    /* atime_correction = atime/atime_next, where atime_next is determined by the detected flicker frequency. */
    /* Vudu.....what if no flicker, what if 2 dominant frequencies, not science */
    atime_correction = 1.0;

    /* Correct ALS counts with channel matching factor and atime correction */
    for (step = 0; step < ALS_NUM_STEPS; step++)
    {
        for (mods = 0; mods < NUM_MODULATORS; mods++)
        {
            als_data[step].mod_normalized_counts[mods] = als_data[step].matching_factors * atime_correction * als_data[step].mod_normalized_counts[mods];
        }
    }

    /* Calculate the Average counts for RGB, WB, and Clear */
    average_counts[RED_PD]       = ((double)(als_data[0].mod_normalized_counts[2] + als_data[2].mod_normalized_counts[2]))/2.0;
    average_counts[GREEN_PD]     = (double)(als_data[0].mod_normalized_counts[1]);
    average_counts[BLUE_PD]      = (double)(als_data[1].mod_normalized_counts[1] + als_data[2].mod_normalized_counts[1])/2.0;
    average_counts[WIDE_BAND_PD] = (double)(als_data[1].mod_normalized_counts[2]);
    average_counts[CLEAR_PD]     = (double)(als_data[0].mod_normalized_counts[0] + als_data[1].mod_normalized_counts[0] + als_data[2].mod_normalized_counts[0])/3.0;

    ir_comp = (average_counts[RED_PD] + average_counts[GREEN_PD] + average_counts[BLUE_PD] - average_counts[CLEAR_PD]) / 2.0;
    ir_comp_ratio = ir_comp / average_counts[CLEAR_PD];

    if (ir_comp_ratio < IR_COMP_RATIO_LO)
    {
        n = N_LO;
    }
    else if ((ir_comp_ratio >= IR_COMP_RATIO_LO) && (ir_comp_ratio < IR_COMP_RATIO_HI))
    {
        n = N_MED;
    }
    else
    {
        n = N_HI;
    }

    if ( log == 1 )
    {
       printf("\n");
       printf("ir_comp_ratio = %f, coeff_index(n) = %d\n", ir_comp_ratio, n);

       printf("\n");
       printf("ALS PDs:\n");
       printf("RED:      %f\n", average_counts[RED_PD]);
       printf("GREEN:    %f\n", average_counts[GREEN_PD]);
       printf("BLUE:     %f\n", average_counts[BLUE_PD]);
       printf("WIDEBAND: %f\n", average_counts[WIDE_BAND_PD]);
       printf("CLEAR:    %f\n", average_counts[CLEAR_PD]);
       printf("\n");
    }
    /* values have already been adjusted for the + 1 */
    atime = calculate_atime_ms();
    //    NRF_LOG_RAW_INFO("atime = %.2f, again = %.2f, ir_comp = %.3f (%d)\n", atime, als_data[STEP_0].mod_normalized_gains[MODULATOR_0], ir_comp_ratio, n);
    if ( log == 1 )
    {
       printf("atime = %f, again = %f, ir_comp = %f (%d)\n", atime, als_data[STEP_0].mod_normalized_gains[MODULATOR_0], ir_comp_ratio, n);
    }

    lux = lux_coeffs[DGF][n] * ((lux_coeffs[RED_COEFF][n]  * average_counts[RED_PD])  + (lux_coeffs[GREEN_COEFF][n] * average_counts[GREEN_PD]) +
                                (lux_coeffs[BLUE_COEFF][n] * average_counts[BLUE_PD]) + (lux_coeffs[WB_COEFF][n]    * average_counts[WIDE_BAND_PD]) +
                                (lux_coeffs[CLEAR_COEFF][n] * average_counts[CLEAR_PD]))/(atime *als_data[STEP_0].mod_normalized_gains[MODULATOR_0]);

    /* calculate cct */
    r_prime = average_counts[RED_PD] - ir_comp;
    if (r_prime == 0)
    {
        r_prime = 1.0;
    }
    b_prime = average_counts[BLUE_PD] - ir_comp;
    cct = (cct_coeffs[COEFA][n] * (b_prime / r_prime)) + cct_coeffs[CTOFFSET][n];

    if ( log == 1 )
    {
       printf("\n");
       printf("ALS Calculated:\n");
       printf("      lux     = %ld\n", (uint32_t)(lux + 0.5));
       printf("      cct     = %ld\n", (uint32_t)(cct + 0.5));
    }
    if (lux < 0)
    {
        lux = 0;
    }
    stat->als.lux = lux;
    stat->als.cct = cct;
    return;
}
 
/* 
 *  Public APIs
 */
void cmd_status_als(ams_device_status_t *stat)
{
  uint8_t log=1;
  als_calc_lux(stat,log);
  printf("------------------------------\n");
  return;
}

void cmd_status_lux(ams_device_status_t *stat)
{
  uint8_t log=0;
  als_calc_lux(stat,log);
}

void cmd_status_cct(ams_device_status_t *stat)
{
  uint8_t log=0;
  als_calc_lux(stat,log);
}
