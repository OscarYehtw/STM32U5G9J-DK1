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

#ifndef __TCS3410_FIFO_H__
#define __TCS3410_FIFO_H__

#define MAX_FIFO_LEN       (512) /* physical FIFO on device */
#define MAX_I2C_READ_LEN    (32) /* maps to linux SMBUS -> I2CSMBUS_BLOCK_MAX */
#define MAX_FIFO_DATA_LEN (1024) /* length of RAM to store device fifo */
#define ALS_DATA_SZ         (36) /* 3 steps 3 modules 9 bytes data, 3 bytes of status for each step*/

#define END_MARKER_SZ        (3)
#define FLCKR_GAIN_SZ        (2) /* number of bytes in the fifo to represent the modulator gains */

#define FD_GAIN0_1_OFFSET    (2)
#define FD_GAIN2_OFFSET      (1)


/* 3 zeros written at the end of the FIFO data before the gains are written */
#define FD_END_MARKER_0      (5)
#define FD_END_MARKER_1      (6)
#define FD_END_MARKER_2      (7)

ams_errno_t sensor_read_fifo(uint8_t *shadow_regs, volatile ams_current_state_t *pcurr_state);
ams_errno_t sensor_fifo_reset(void);
ams_errno_t sensor_process_fifo(uint8_t *shadow_regs, volatile ams_current_state_t *pcurr_state);

#endif /* __TCS3410_FIFO_H__ */

