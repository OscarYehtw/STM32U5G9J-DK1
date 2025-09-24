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

#ifndef __AMS_ERRNO_H__
#define __AMS_ERRNO_H__

/* The typedef enum must match the order of the strings defined */
/* in ams_errno.c --> static const char* const ams_errno_str[] */
typedef enum 
{
    /* No error */
    AMS_SUCCESS    = 0,

    /* CLI Command failed to process */
    AMS_CLI_FAILURE    ,

    /* Error Initing i2c */
    AMS_I2C_INIT_ERROR ,

    /* AMS Device init failure */
    AMS_DEVICE_INIT_FAILURE,

    /* Cannot validate device */
    AMS_DEVICE_VALIDATE_ERROR,

    /* Null pointer */
    AMS_DEVICE_NULL_PTR, 

    /* current software not supported index of device */
    AMS_DEVICE_OUT_OF_BOUNDS,

    /* FIFO not empty */
    AMS_DEVICE_FIFO_NOT_EMPTY,

    /* Serial number of device does not match the one specified in calibration header */
    AMS_INVALID_CALIBRATION_DATA,

    /* FIFO End Marker not present */
    AMS_NO_END_MARKER,

    /* Unable to decode compressed flicker data */
    AMS_COMPRESS_DECODE_FAILURE, 

    /* Cannot calculate a flicker frequency */
    AMS_FLICKER_FAILURE,

    /* ams fft could not be calculated */
    AMS_FFT_FAILURE,

    /* */
    AMS_UNKNOWN_ERROR,

    /* */
    AMS_LAST_ERROR = AMS_UNKNOWN_ERROR + 1,
} ams_errno_t;

const char *ams_errno_code_2_str(ams_errno_t err);

#endif /* __AMS_ERRNO_H__ */
