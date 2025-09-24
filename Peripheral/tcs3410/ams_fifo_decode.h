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

#ifndef __AMS_FIFO_DECODE_H__
#define __AMS_FIFO_DECODE_H__

/* supported fifo formats  */
typedef enum _e_fifo_data_format{
	FIFO_FORMAT_UNCOMPRESSED = 1,
	FIFO_FORMAT_DIFFERENCE,
	FIFO_FORMAT_COMPRESSED,
	FIFO_FORMAT_DIFFERENCE_COMPRESSED,
	FIFO_FORMAT_MULTI_CHL_COMPRESSED,
	FIFO_FORMAT_MULTI_CHL_DIFFERENCE_COMPRESSED
}ams_fifo_format_t;

/* supported fifo decoded results  */
typedef enum _e_fifo_decode_result{
	FIFO_DECODE_SUCCESS,
	FIFO_DECODE_FAILURE,
	FIFO_DECODE_UNSUPPORTED_FORMAT,
	FIFO_DECODE_NULL_PARAM_ERROR
}ams_fifo_decode_result_t;

ams_fifo_decode_result_t fifo_data_decode(void *input, void *output, int num_channels, uint16_t num_bytes, uint16_t output_sz_bytes, int *packet_size, ams_fifo_format_t data_format);

#endif /* __AMS_FIFO_DECODE_H__ */
