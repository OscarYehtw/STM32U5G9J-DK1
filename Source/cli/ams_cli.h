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

#ifndef __AMS_CLI_H__
#define __AMS_CLI_H__

/* Define CLI prompt */
#define AMS_CLI_PROMPT        "tcs3410{als,fd}$  "


#define AMS_CLI_ALS_FEATURE      "als"
#define AMS_CLI_FLCKR_FEATURE    "flckr"
#define AMS_CLI_ALL_FEATURE       "all"

#define AMS_CLI_SAI_ENABLE          "enable"
#define AMS_CLI_SAI_DISABLE         "disable"
#define AMS_CLI_SAI_CLEAR           "clear"

#define AMS_CLI_STATUS_ALS           "als"
#define AMS_CLI_STATUS_FLCKR         "flckr"
#define AMS_CLI_STATUS_FIFO          "fifo"

#define AMS_CLI_STATUS_FREQ         "freq"
#define AMS_CLI_STATUS_LUX          "lux"
#define AMS_CLI_STATUS_CCT          "cct"
#define AMS_CLI_STATUS_UV           "uv"  /* uv index */


#define AMS_CLI_ON                  "on"
#define AMS_CLI_OFF                 "off"

#define AMS_CLI_BUFFER_SIZE       (20)

typedef struct _limits
{
    uint16_t min;
    uint16_t max;
}config_param_limits_t;

typedef enum
{
    PARAM_SAMPLE_TIME    ,
    PARAM_MOD_TRIGGER    ,
    PARAM_WAIT_TIME      ,
    PARAM_AGC_MODE       ,
    PARAM_AGC_NR_SAMPLES ,
    PARAM_FIFO_RESET     ,
    PARAM_FIFO_THRESHOLD ,
    PARAM_ALS_NR_SAMPLES ,
    PARAM_FD_NR_SAMPLES  ,
} config_parameter_types_t;


#endif  /* __AMS_CLI_H__*/
