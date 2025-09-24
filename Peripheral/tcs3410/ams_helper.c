/*
 *****************************************************************************
 * Copyright by ams AG                                                       *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
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

/*
 *  DESCRIPTION:
 *
 *  Helper utility to convert unsigned 16 bit numbers to signed 16 bits numbers
 *  to be processed by ams' fft algorithm.  Most ams devices produce unsigned
 *  16 bit input data.  However, the fft algorithm requires signed 16 bit data.
 *  If the unsigned data is greater than 32767 (0x7FFF), the "sign" bit will be
 *  set and cause undesirable results within the FFT algorithm.  Use this
 *  utility to normalize the data so that the input data is not greater than 
 *  32767.
 *
 */
 
#include "ams_helper.h"

int ams_normalize_fft_data(uint16_t *input, uint16_t *output, uint32_t size)
{
    uint32_t idx;
    uint16_t max = 0;
    
    if ((input == NULL) || (output == NULL))
    {
        return(-1);
    }

    /* Find the max value from within the input data */
    for (idx = 0; idx < size; idx++)
    {
        if (input[idx] > max)
        {
            max = input[idx];
        }
    }

    /* Normalize the unsigned data by max/2 to fit within a 16 signed int */
    for (idx = 0; idx < size; idx++)
    {
        output[idx] = (input[idx] - (max / 2));
    }

    return(0);
}
