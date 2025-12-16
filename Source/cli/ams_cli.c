/*----------------------------------------------------------------------------
 *      Name:    ams_cli.c
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
#include "ams_cli.h"
#include "ams_device.h"
#include "tcs3410_hwdef.h"
#include "tcs3410.h"
#include "tcs3410_als.h"
#include "ams_cli_als.h"

#define CLI_BUFFER_SIZE     2048
static  char g_CLI_buffer[CLI_BUFFER_SIZE] = {0};
#define MAX_READ_REGS              (40)

static const config_param_limits_t cfg_limits[] =
{
    /* START - Base Parameters */
    [PARAM_SAMPLE_TIME]    = 
                             {
                              .min = 0,
                              .max = 0x7FF,
                             },
    [PARAM_MOD_TRIGGER]    =
                             {
                               .min = 0,
                               .max = 4, /* actual limit is 5 but SW does not */
                             },          /* vsync - see datasheet             */
    [PARAM_WAIT_TIME]      = 
                             {
                              .min = 0,
                              .max = 0xFF,
                             },
    [PARAM_AGC_MODE ]      = 
                             {
                              .min = 0,
                              .max = 3,
                             },
    [PARAM_AGC_NR_SAMPLES] = 
                             {
                              .min = 0,
                              .max = 0x7FF,
                             },
    [PARAM_FIFO_RESET] = 
                             {
                              .min = 0,
                              .max = 1,
                             },
    [PARAM_FIFO_THRESHOLD] = 
                             {
                              .min = 0,
                              .max = 0x1FF,
                             },
    /* END - Base Parameters */

    /* START - ALS Parameters */
    [PARAM_ALS_NR_SAMPLES] = 
                             {
                              .min = 0,
                              .max = 0x7FF,
                             },
    /* END - ALS Parameters */

    /* START - FD Parameters */
    [PARAM_FD_NR_SAMPLES] = 
                             {
                              .min = 0,
                              .max = 0x7FF,
                             },
    /* END - FD Parameters */

};
#define CHECK_LIMITS(idx, val)                                                 \
       ({                                                                      \
           bool _ret_val = true;                                               \
           do                                                                  \
           {                                                                   \
               if ((val < cfg_limits[idx].min) || (val > cfg_limits[idx].max)) \
               {                                                               \
                   _ret_val = false;                                           \
               }                                                               \
           } while(0);                                                         \
           _ret_val;                                                           \
       })

/* must match ams_agc_mode_t */
static const char * const agcmode2_str[] =
{
   "Disabled",
   "ASAT Enabled",
   "Predictive AGC Enabled",
   "Both ASAT and Predictive Enabled",
};

static const float mod_trigger_timing_ms[] =
{
    0.0      , /* OFF      */
    2.844    , /* normal   */
    45.511   , /* long     */
    0.088889 , /* fast     */
    1.422    , /* fastlong */
    -1.0     , /* vsync - not used */
};

static bool cmd_config_limit_check(ams_config_feature_t cfg_type, ams_sensor_config_t *cfg)
{
    bool ret_val = true;

    if (cfg == NULL)
    {
        return(false);
    }
    
    switch(cfg_type)
    {
        case AMS_CONFIG_BASE:
        {
            if (!(CHECK_LIMITS(PARAM_SAMPLE_TIME,    cfg->sample_time)   &&
                  CHECK_LIMITS(PARAM_MOD_TRIGGER,     cfg->mod_trigger)  &&
                  CHECK_LIMITS(PARAM_WAIT_TIME,      cfg->wait_time)     &&
                  CHECK_LIMITS(PARAM_AGC_MODE,       cfg->agc_mode)      &&
                  CHECK_LIMITS(PARAM_AGC_NR_SAMPLES, cfg->agc_nr_samples)))
            {
                ret_val = false;
            }
            break;
        }
        case AMS_CONFIG_ALS:
        {
            if (!CHECK_LIMITS(PARAM_ALS_NR_SAMPLES, cfg->als_nr_samples))
            {
                ret_val = false;
            }
            break;
        }
        case AMS_CONFIG_FD:
        {
            if (!CHECK_LIMITS(PARAM_FD_NR_SAMPLES, cfg->fd_nr_samples))
            {
                ret_val = false;
            }
            break;
        }
        case AMS_CONFIG_FIFO:
        {
            if (!(CHECK_LIMITS(PARAM_FIFO_RESET,     cfg->fifo_reset)   &&
                  CHECK_LIMITS(PARAM_FIFO_THRESHOLD, cfg->fifo_threshold)))
            {
                ret_val = false;
            }
            break;
        }
        default:
        {
            break;
        }
    }

    return(ret_val);
}

static ams_errno_t process_cmd_enable_disable(const char *pFeature, ams_feature_enable_t en)
{
    ams_feature_t feature = AMS_FEATURES_END;
    ams_errno_t ret = AMS_SUCCESS;

    if (!strncmp(pFeature, AMS_CLI_ALS_FEATURE, strlen(AMS_CLI_ALS_FEATURE)))
    {
        feature = AMS_FEATURE_ALS;
    }
    else if (!strncmp(pFeature, AMS_CLI_FLCKR_FEATURE, strlen(AMS_CLI_FLCKR_FEATURE)))
    {
        feature = AMS_FEATURE_FLICKER;
    }
    else if (!strncmp(pFeature, AMS_CLI_ALL_FEATURE, strlen(AMS_CLI_ALL_FEATURE)))
    {
        feature = AMS_FEATURE_ALL;
    }
    else
    {
        printf("Invalid parameter\r\n");
        ret = AMS_CLI_FAILURE;
    }

    if (feature != AMS_FEATURES_END)
    {
        ret = ams_device_enable(feature, en);
    }
    return(ret);
}

static ams_errno_t process_cmd_sai(const char *pstate)
{
    ams_sai_state_t state = AMS_SAI_END;
    ams_errno_t ret = AMS_SUCCESS;

    if (!strncmp(pstate, AMS_CLI_SAI_ENABLE, strlen(AMS_CLI_SAI_ENABLE)))
    {
        state = AMS_SAI_ENABLE;
    }
    else if (!strncmp(pstate, AMS_CLI_SAI_DISABLE, strlen(AMS_CLI_SAI_DISABLE)))
    {
        state = AMS_SAI_DISABLE;
    }
    else if (!strncmp(pstate, AMS_CLI_SAI_CLEAR, strlen(AMS_CLI_SAI_CLEAR)))
    {
        state = AMS_SAI_CLEAR;
    }
    else
    {
        ret = AMS_CLI_FAILURE;
        printf("Invalid parameter\r\n");
    }

    if (state != AMS_SAI_END)
    {
        ret = ams_device_sai(state);
    }
    return(ret);
}

static ams_errno_t process_pon(const char *on_off)
{
    bool state = false;
    ams_errno_t ret = AMS_SUCCESS;

    if (!strncmp(on_off, AMS_CLI_ON, strlen(AMS_CLI_ON)))
    {
        state = true;
    }
    else if (!strncmp(on_off, AMS_CLI_OFF, strlen(AMS_CLI_OFF)))
    {
        state = false;
    }
    else
    {
        printf("Invalid parameter\r\n");
        return(AMS_CLI_FAILURE);
    }

    ret = ams_device_pon(state);

    return(ret);
}

/*----------------------------------------------------------------------------
 *        cmd_enable  — Enable ALS / Flicker / All
 *---------------------------------------------------------------------------*/
void cmd_enable(char *par)
{
    char *next;
    char *pFeature;

    /* No parameter */
    if (par == NULL || *par == 0)
    {
        printf("Usage: enable <als|flckr|all>\r\n");
        return;
    }

    /* Parse feature */
    pFeature = get_entry(par, &next);
    if (pFeature == NULL)
    {
        printf("Missing feature: als|flckr|all\r\n");
        return;
    }

    /* Check sensor status */
    if (!sensor_get_validated())
    {
        printf("Command Failed: Sensor not initialized or disconnected\r\n");
        return;
    }

    /* Execute enable */
    if (process_cmd_enable_disable((const char *)pFeature, AMS_FEATURE_ENABLE) != AMS_SUCCESS)
    {
        printf("Command Failed: enable %s\r\n", pFeature);
        return;
    }

    printf("Enabled: %s\r\n", pFeature);
}

/*----------------------------------------------------------------------------
 *        cmd_disable  — Disable ALS / Flicker / All
 *---------------------------------------------------------------------------*/
void cmd_disable(char *par)
{
    char *next;
    char *pFeature;

    /* No parameter */
    if (par == NULL || *par == 0)
    {
        printf("Usage: disable <als|flckr|all>\r\n");
        return;
    }

    /* Parse feature */
    pFeature = get_entry(par, &next);
    if (pFeature == NULL)
    {
        printf("Missing feature: als|flckr|all\r\n");
        return;
    }

    /* Check sensor validity */
    if (!sensor_get_validated())
    {
        printf("Command Failed: Sensor not initialized or disconnected\r\n");
        return;
    }

    /* Execute disable */
    if (process_cmd_enable_disable((const char *)pFeature, AMS_FEATURE_DISABLE) != AMS_SUCCESS)
    {
        printf("Command Failed: disable %s\r\n", pFeature);
        return;
    }

    printf("Disabled: %s\r\n", pFeature);
}

/*----------------------------------------------------------------------------
 *        cmd_sai  — Control Sleep After Interrupt (SAI)
 *---------------------------------------------------------------------------*/
void cmd_sai(char *par)
{
    char *next;
    char *pArg;

    /* No args → usage */
    if (par == NULL || *par == 0)
    {
        printf("Usage: sai <clear|enable|disable>\r\n");
        return;
    }

    /* Parse argument */
    pArg = get_entry(par, &next);
    if (pArg == NULL)
    {
        printf("Missing argument: clear|enable|disable\r\n");
        return;
    }

    /* Check sensor status */
    if (!sensor_get_validated())
    {
        printf("Command Failed: Sensor not initialized or disconnected\r\n");
        return;
    }

    /* Execute SAI command */
    if (process_cmd_sai((const char *)pArg) != AMS_SUCCESS)
    {
        printf("Command Failed: sai %s\r\n", pArg);
        return;
    }

    printf("SAI: %s\r\n", pArg);
}

/*----------------------------------------------------------------------------
 *        cmd_pon  — Power On/Off control
 *---------------------------------------------------------------------------*/
void cmd_pon(char *par)
{
    char *next;
    char *pArg;

    /* No args → usage */
    if (par == NULL || *par == 0)
    {
        printf("Usage: pon <on|off>\r\n");
        return;
    }

    /* Parse argument */
    pArg = get_entry(par, &next);
    if (pArg == NULL)
    {
        printf("Missing argument: on|off\r\n");
        return;
    }

    /* Check sensor state */
    if (!sensor_get_validated())
    {
        printf("Command Failed: Sensor not initialized or disconnected\r\n");
        return;
    }

    /* Execute PON command */
    if (process_pon((const char *)pArg) != AMS_SUCCESS)
    {
        printf("Command Failed: pon %s\r\n", pArg);
        return;
    }

    printf("PON: %s\r\n", pArg);
}

/*----------------------------------------------------------------------------
 *        cmd_config  — Base sensor configuration
 *---------------------------------------------------------------------------*/
void cmd_config(char *par)
{
    char *next;
    char *p1, *p2, *p3, *p4, *p5;

    ams_sensor_config_t cfg = {0};

    if (!sensor_get_validated())
    {
        printf("Command Failed: Sensor not initialized or disconnected\r\n");
        return;
    }

    if (par == NULL || *par == 0)
    {
        printf("Usage: config <sample_time> <mod_trigger> <wait_time> <agc_mode> <agc_nr_samples>\r\n");
        return;
    }

    /* Parse all 5 parameters */
    p1 = get_entry(par, &next);
    p2 = get_entry(next, &next);
    p3 = get_entry(next, &next);
    p4 = get_entry(next, &next);
    p5 = get_entry(next, &next);

    if (!p1 || !p2 || !p3 || !p4 || !p5)
    {
        printf("Incorrect parameters\r\n");
        return;
    }

    cfg.sample_time    = (uint16_t)strtoul(p1, NULL, 10);
    cfg.mod_trigger    = (uint16_t)strtoul(p2, NULL, 10);
    cfg.wait_time      = (uint16_t)strtoul(p3, NULL, 10);
    cfg.agc_mode       = (uint16_t)strtoul(p4, NULL, 10);
    cfg.agc_nr_samples = (uint16_t)strtoul(p5, NULL, 10);

    if (!cmd_config_limit_check(AMS_CONFIG_BASE, &cfg))
    {
        printf("Command Failed: Parameters out of bound\r\n");
        return;
    }

    if (ams_device_configure(AMS_CONFIG_BASE, (void *)&cfg) != AMS_SUCCESS)
    {
        printf("Command Failed: config\r\n");
        return;
    }

    printf("Configured base: sample=%u, trig=%u, wait=%u, agc=%u, agc_samples=%u\r\n",
           cfg.sample_time, cfg.mod_trigger, cfg.wait_time, cfg.agc_mode, cfg.agc_nr_samples);
}

/*----------------------------------------------------------------------------
 *        cmd_config_als  — ALS configuration
 *---------------------------------------------------------------------------*/
void cmd_config_als(char *par)
{
    char *next;
    char *p1;

    ams_sensor_config_t cfg = {0};

    if (!sensor_get_validated())
    {
        printf("Command Failed: Sensor not initialized or disconnected\r\n");
        return;
    }

    if (par == NULL || *par == 0)
    {
        printf("Usage: config_als <als_nr_samples>\r\n");
        return;
    }

    p1 = get_entry(par, &next);
    if (!p1)
    {
        printf("Missing als_nr_samples\r\n");
        return;
    }

    cfg.als_nr_samples = (uint16_t)strtoul(p1, NULL, 10);

    if (!cmd_config_limit_check(AMS_CONFIG_ALS, &cfg))
    {
        printf("Command Failed: Parameters out of bound\r\n");
        return;
    }

    if (ams_device_configure(AMS_CONFIG_ALS, (void *)&cfg) != AMS_SUCCESS)
    {
        printf("Command Failed: config_als\r\n");
        return;
    }

    printf("Configured ALS: samples=%u\r\n", cfg.als_nr_samples);
}

/*----------------------------------------------------------------------------
 *        cmd_config_fd  — Flicker Detector config
 *---------------------------------------------------------------------------*/
void cmd_config_fd(char *par)
{
    char *next;
    char *p1;

    ams_sensor_config_t cfg = {0};

    if (!sensor_get_validated())
    {
        printf("Command Failed: Sensor not initialized or disconnected\r\n");
        return;
    }

    if (par == NULL || *par == 0)
    {
        printf("Usage: config_fd <fd_nr_samples>\r\n");
        return;
    }

    p1 = get_entry(par, &next);
    if (!p1)
    {
        printf("Missing fd_nr_samples\r\n");
        return;
    }

    cfg.fd_nr_samples = (uint16_t)strtoul(p1, NULL, 10);

    if (!cmd_config_limit_check(AMS_CONFIG_FD, &cfg))
    {
        printf("Command Failed: Parameters out of bound\r\n");
        return;
    }

    if (ams_device_configure(AMS_CONFIG_FD, (void *)&cfg) != AMS_SUCCESS)
    {
        printf("Command Failed: config_fd\r\n");
        return;
    }

    printf("Configured FD: samples=%u\r\n", cfg.fd_nr_samples);
}

/*----------------------------------------------------------------------------
 *        cmd_config_fifo  — Configure AMS FIFO
 *---------------------------------------------------------------------------*/
void cmd_config_fifo(char *par)
{
    char *next;
    char *pReset;
    char *pThr;

    unsigned int reset = 0;
    unsigned int threshold = 0;

    ams_sensor_config_t cfg = {0};

    /* Check AMS sensor state */
    if (!sensor_get_validated())
    {
        printf("Command Failed: Sensor not initialized or disconnected\r\n");
        return;
    }

    /* Check input */
    if (par == NULL || *par == 0)
    {
        printf("Usage: fifo <fifo_reset 0/1> <threshold 0~2047>\r\n");
        return;
    }

    /* Parse fifo_reset */
    pReset = get_entry(par, &next);
    if (pReset == NULL)
    {
        printf("Missing fifo_reset parameter\r\n");
        return;
    }

    if (sscanf(pReset, "%u", &reset) != 1 || reset > 1)
    {
        printf("Invalid fifo_reset (must be 0 or 1)\r\n");
        return;
    }

    /* Parse fifo_threshold */
    pThr = get_entry(next, &next);
    if (pThr == NULL)
    {
        printf("Missing fifo_threshold parameter\r\n");
        return;
    }

    if (sscanf(pThr, "%u", &threshold) != 1 || threshold > 2047)
    {
        printf("Invalid fifo_threshold (must be 0~2047)\r\n");
        return;
    }

    /* Fill AMS config structure */
    cfg.fifo_reset     = (uint16_t)reset;
    cfg.fifo_threshold = (uint16_t)threshold;

    /* Run limit check */
    if (!cmd_config_limit_check(AMS_CONFIG_FIFO, &cfg))
    {
        printf("Command Failed: Parameters out of valid range\r\n");
        return;
    }

    /* Apply configuration */
    if (ams_device_configure(AMS_CONFIG_FIFO, (void *)&cfg) != AMS_SUCCESS)
    {
        printf("Command Failed: config_fifo (%u, %u) failed\r\n",
               reset, threshold);
        return;
    }

    /* Success */
    printf("FIFO configured: reset=%u, threshold=%u\r\n",
           reset, threshold);
}

void cmd_setup(char *par)
{
    ams_sensor_config_t cfg = {0};
    ams_errno_t ret;

    if (!sensor_get_validated())
    {
        printf("Command Failed: Sensor not initialized or not connected\r\n");
        return;
    }

    /* No arguments allowed */
    if (par && *par)
    {
        printf("Usage: setup\r\n");
        return;
    }

    ret = ams_device_setup((void *)&cfg);

    if (ret == AMS_SUCCESS)
    {
        printf("Current Device Setup\n");
        printf("---------------------\n");

        printf("Sample Time    = %u (%.3f ms)\r\n",
            cfg.sample_time,
            (float)((cfg.sample_time + 1) * MOD_CLOCK_STEP_MS));

        printf("Mod Trigger    = %u (%.3f ms)\r\n",
            cfg.mod_trigger,
            mod_trigger_timing_ms[cfg.mod_trigger]);

        printf("Wait Time*     = %u (%.3f ms)\r\n",
            cfg.wait_time,
            (float)((cfg.wait_time + 1) * mod_trigger_timing_ms[cfg.mod_trigger]));

        printf("AGC Mode       = %u (%s)\r\n",
            cfg.agc_mode, agcmode2_str[cfg.agc_mode]);

        printf("AGC Samples    = %u\r\n", cfg.agc_nr_samples);
        printf("ALS Samples    = %u\r\n", cfg.als_nr_samples);
        printf("FD Samples     = %u\r\n", cfg.fd_nr_samples);
        printf("FIFO Threshold = %u\r\n", cfg.fifo_threshold);
    }
}

void cmd_id(char *par)
{
    if (!sensor_get_validated())
    {
        printf("Command Failed: Sensor not initialized or not connected\r\n");
        return;
    }

    if (par && *par)
    {
        printf("Usage: id\r\n");
        return;
    }

    ams_device_read(REG_AUXID, (uint8_t *)&g_CLI_buffer[0], 3);

    uint8_t dev  = (g_CLI_buffer[2] & DEVICE_ID_MASK) >> DEVICE_ID_SHIFT;
    uint8_t rev1 = (g_CLI_buffer[1] & REV_ID_MASK)    >> REV_ID_SHIFT;
    uint8_t rev2 = (g_CLI_buffer[0] & AUX_ID_MASK)    >> AUX_ID_SHIFT;

    printf("Device ID  : 0x%02X (%u)\r\n", dev,  dev);
    printf("REV ID     : 0x%02X (%u)\r\n", rev1, rev1);
    printf("REV2 ID    : 0x%02X (%u)\r\n", rev2, rev2);
}

/*----------------------------------------------------------------------------
 *        cmd_dump  — dump
 *---------------------------------------------------------------------------*/
void cmd_dump(char *par)
{
    if (par && *par)
    {
        printf("Usage: dump\r\n");
        return;
    }

    if (!sensor_get_validated())
    {
        printf("Command Failed: Device failed to initialize or not connected\r\n");
        return;
    }

    memset(g_CLI_buffer, 0, CLI_BUFFER_SIZE);
    ams_registers_get(g_CLI_buffer, CLI_BUFFER_SIZE);
    printf("%s\r\n", g_CLI_buffer);
}

void cmd_isUP(char *par)
{
    ams_errno_t ret;
    bool ok = false;

    if (!sensor_get_validated())
    {
        printf("Command Failed: Sensor not initialized or not connected\r\n");
        return;
    }

    if (par && *par)
    {
        printf("Usage: up\r\n");
        return;
    }

    ret = ams_device_isUP(&ok);

    if (ret == AMS_SUCCESS)
    {
        if (ok)
            printf("Device is operational\r\n");
        else
            printf("Device is NOT operational / wrong device\r\n");
    }
}

static void cmd_status_flckr(ams_device_status_t *stat)
{
    printf("\nFlicker Status:\n");
    printf("End Marker Detected = %s\n", stat->fd.end_marker == true ? "YES" : "NO");
    printf("Gain (Step 1 Mod 0) = 0x%02X (%dx)\n", stat->fd.gain_reg, stat->fd.gain_mod);
    printf("Data Length         = %d bytes\n", stat->fd.len);
    printf("Last Detected       = %.2f Hz.\n", stat->fd.freq);

    return;
}

static void cmd_status_fifo(ams_device_status_t *stat)
{
    printf("\nFIFO Status:\n");
    printf("Last Level =  %3d\n", stat->fifo.level);
    printf("Threshold  =  %3d\n", stat->fifo.threshold);
    printf("Overflows  =  %3ld\n", stat->fifo.overflow);
    printf("Underflows =  %3ld\n", stat->fifo.underflow);

    return;
}

static void cmd_status_general(ams_device_status_t *stat)
{
    printf("Device Status\n------------------------------\n");
    printf("PON           : %s\n", ((stat->pon 	 == true) ? "On" : "Off"));
    printf("Log Irq       : %s\n", ((stat->log_irq  == true) ? "Enabled" : "Disabled"));
    printf("Flicker       : %s\n", ((stat->fd_en	 == AMS_FEATURE_ENABLE) ? "Enabled" : "Disabled"));
    printf("ALS           : %s\n", ((stat->als_en	 == AMS_FEATURE_ENABLE) ? "Enabled" : "Disabled"));
    printf("SAI           : %s (%s)\n", ((stat->sai.sai == AMS_SAI_ENABLE) ? "Enabled" : "Disabled"),
                                        ((stat->sai.active == true) ? "Active"	: "Not Active"));

    return;
}

static void cmd_status_specific(char *type, ams_device_status_t *stat)
{
    if (!type)
    {
        printf("Command Failed: status <als|flckr|fifo|freq|lux|cct>\r\n");
        return;
    }

    /* ALS */
    if (!strncmp(type, AMS_CLI_STATUS_ALS, strlen(AMS_CLI_STATUS_ALS)))
    {
        cmd_status_general(stat);
        cmd_status_als(stat);
    }
    /* Flicker */
    else if (!strncmp(type, AMS_CLI_STATUS_FLCKR, strlen(AMS_CLI_STATUS_FLCKR)))
    {
        cmd_status_general(stat);
        cmd_status_flckr(stat);
    }
    /* FIFO */
    else if (!strncmp(type, AMS_CLI_STATUS_FIFO, strlen(AMS_CLI_STATUS_FIFO)))
    {
        cmd_status_general(stat);
        cmd_status_fifo(stat);
    }
    /* Frequency only */
    else if (!strncmp(type, AMS_CLI_STATUS_FREQ, strlen(AMS_CLI_STATUS_FREQ)))
    {
        printf("%.3f\n", stat->fd.freq);
    }
    /* Lux only */
    else if (!strncmp(type, AMS_CLI_STATUS_LUX, strlen(AMS_CLI_STATUS_LUX)))
    {
        cmd_status_lux(stat);
        printf("%lu\n", (uint32_t)(stat->als.lux + 0.5));
    }
    /* CCT only */
    else if (!strncmp(type, AMS_CLI_STATUS_CCT, strlen(AMS_CLI_STATUS_CCT)))
    {
        cmd_status_cct(stat);
        printf("%lu\n", (uint32_t)(stat->als.cct + 0.5));
    }
    /* Unknown */
    else
    {
        printf("Command Failed: status <als|flckr|fifo|freq|lux|cct>\r\n");
    }
}

void cmd_status(char *par)
{
    ams_device_status_t stat;
    ams_errno_t ret;

    if (!sensor_get_validated())
    {
        printf("Command Failed: Sensor not initialized or not connected\r\n");
        return;
    }

    if (par == NULL || *par == 0)
    {
        printf("Usage: status <als|flckr|fifo|freq|lux|cct>\r\n");
        return;
    }

    /* get sub-category */
    char *next;
    char *pType = get_entry(par, &next);

    if (!pType)
    {
        printf("Usage: status <als|flckr|fifo|freq|lux|cct>\r\n");
        return;
    }

    ret = ams_device_status((void *)&stat);

    if (ret != AMS_SUCCESS)
    {
        printf("Command Failed: status\r\n");
        return;
    }

    /* Hand off to a helper (same as original code) */
    cmd_status_specific(pType, &stat);
}

static ams_errno_t process_log_irq(char *on_off)
{
    bool state = false;
    ams_errno_t ret = AMS_SUCCESS;

    if (!strncmp(on_off, AMS_CLI_ON, strlen(AMS_CLI_ON)))
    {
        state = true;
    }
    else if (!strncmp(on_off, AMS_CLI_OFF, strlen(AMS_CLI_OFF)))
    {
        state = false;
    }
    else
    {
        return(AMS_CLI_FAILURE);
    }

    ret = ams_device_log_irq(state);

    return(ret);
}

static char *cli_trim(char *s)
{
    char *end;

    /* skip leading spaces */
    while (*s == ' ' || *s == '\t') s++;

    if (*s == 0)
        return s;

    /* trim trailing spaces */
    end = s + strlen(s) - 1;
    while (end > s && (*end == ' ' || *end == '\t' ||
                       *end == '\r' || *end == '\n'))
    {
        *end-- = 0;
    }

    return s;
}

/*----------------------------------------------------------------------------
 *        cmd_version  — version
 *---------------------------------------------------------------------------*/
void cmd_version(char *par)
{
    if (par && *par)
    {
        printf("Usage: version\r\n");
        return;
    }

    ams_device_get_version();
}

/*----------------------------------------------------------------------------
 *        cmd_irq  — irq <on|off>
 *---------------------------------------------------------------------------*/
void cmd_irq(char *par)
{
    char *arg;

    if (par == NULL || *par == 0)
    {
        printf("Command Failed: irq: Incorrect number of parameters\r\n");
        printf("Usage: irq <on|off>\r\n");
        return;
    }

    arg = cli_trim(par);

    if (!sensor_get_validated())
    {
        printf("Command Failed: Device failed to initialize or not connected\r\n");
        return;
    }

    if (strcmp(arg, "on") != 0 && strcmp(arg, "off") != 0)
    {
        printf("Usage: irq <on|off>\r\n");
        return;
    }

    if (process_log_irq(arg) != AMS_SUCCESS)
    {
        printf("Command Failed: irq\r\n");
    }
}
