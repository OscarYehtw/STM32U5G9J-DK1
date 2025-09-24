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

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "ams_errno.h"
#include "tcs3410_hwdef.h"
#include "tcs3410.h"
#include "tcs3410_als.h"
#include "tcs3410_utils.h"
#include "master_i2c.h"
#include "ams_device.h"
#include "ams_platform.h"

/******************************************************************************/
/*                                                                            */
/*                       Global APIs                                          */
/*                                                                            */
/******************************************************************************/ 
ams_errno_t process_als_data(volatile ams_current_state_t *pcurr_state, uint8_t *pfifo)
{
    ams_errno_t ret_val = AMS_SUCCESS;

    if (pfifo == NULL)
    {
        return(AMS_DEVICE_NULL_PTR);
    }

    /* Save the als fifo data */
    uint16_t idx;
    for (idx = 1; idx <= ALS_FIFO_DATA_LENGTH; idx++)
    {
      pcurr_state->als.als_fifo_data[idx-1] = pfifo[idx-1];
    }

#if defined(DEBUG_SHOW_ALS_FIFO)
    //NRF_LOG_RAW_INFO("In process als fifo data \n");
    for (idx = 1; idx <= ALS_FIFO_DATA_LENGTH; idx++)
    {
      //NRF_LOG_RAW_INFO("%2x ", pcurr_state->als.als_fifo_data[idx-1]);
      if ((idx % 9) == 0)
      {
         NRF_LOG_RAW_INFO("\n");
      }
    }
    //NRF_LOG_RAW_INFO("\n");
#endif


    /* lux is calculated within the CLI */

    return(ret_val);
}
