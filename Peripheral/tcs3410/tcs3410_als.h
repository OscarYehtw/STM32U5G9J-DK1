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

#ifndef __TCS3410_ALS_H__
#define __TCS3410_ALS_H__

#define ALS_STATUS_MOD0_GAIN_SHIFT (0)
#define ALS_STATUS_MOD0_GAIN_MASK  (0x0F)

#define ALS_STATUS_MOD1_GAIN_SHIFT (4)
#define ALS_STATUS_MOD1_GAIN_MASK  (0xF0)

#define ALS_STATUS_MOD2_GAIN_SHIFT (0)
#define ALS_STATUS_MOD2_GAIN_MASK (0x0F)

/* sequence step */
#define ALS_STATUS_MEAS_SEQ_STEP_SHIFT   (6)
#define ALS_STATUS_MEAS_SEQ_STEP_MASK    (3 << ALS_STATUS_MEAS_SEQ_STEP_SHIFT)

/* analog status on modulators */
#define ALS_STATUS_ANA_SAT_MOD0_SHIFT   (5)
#define ALS_STATUS_ANA_SAT_MOD0_MASK    (1 << ALS_STATUS_ANA_SAT_MOD0_SHIFT)

#define ALS_STATUS_ANA_SAT_MOD1_SHIFT   (4)
#define ALS_STATUS_ANA_SAT_MOD1_MASK    (1 << ALS_STATUS_ANA_SAT_MOD1_SHIFT)

#define ALS_STATUS_ANA_SAT_MOD2_SHIFT   (3)
#define ALS_STATUS_ANA_SAT_MOD2_MASK    (1 << ALS_STATUS_ANA_SAT_MOD2_SHIFT)


ams_errno_t process_als_data(volatile ams_current_state_t *pcurr_state, uint8_t *pfifo);


#endif /* __TCS3410_ALS_H__ */

