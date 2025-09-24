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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ams_errno.h"

/* This must match the order defined in ams_errno.h */
static const char* const ams_errno_str[] =
{
    "AMS_SUCCESS",
    "AMS_CLI_FAILURE",
    "AMS_I2C_INIT_ERROR",
    "AMS_DEVICE_INIT_FAILURE",
    "AMS_DEVICE_FAILED_VALIDATION",
    "AMS_DEVICE_NULL_PTR",
    "AMS_DEVICE_OUT_OF_BOUNDS",
    "AMS_DEVICE_FIFO_NOT_EMPTY",
    "AMS_INVALID_CALIBRATION_DATA",
    "AMS_NO_END_MARKER",
    "AMS_COMPRESS_DECODE_FAILURE",
    "AMS_FLICKER_FAILURE",
    "AMS_FFT_FAILURE",
    "AMS_UNKNOWN_ERROR",
};

const char *ams_errno_code_2_str(ams_errno_t err)
{
    const char *err_str = NULL;
    if (err < AMS_LAST_ERROR)
    {
        err_str = ams_errno_str[err];
    }

    return(err_str);
}
