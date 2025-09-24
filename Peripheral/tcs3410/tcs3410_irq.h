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

#ifndef __TCS3410_IRQ_H__
#define __TCS3410_IRQ_H__

enum status_mask
{
    STATUS_MINT    = (1 << 7),   /* modulator */
    STATUS_AINT    = (1 << 3),   /* ALS */
    STATUS_FINT    = (1 << 2),   /* FIFO */
    STATUS_SINT    = (1 << 0),   /* System */
};

enum status2_mask
{
    STATUS2_ALS_DATA_VALID  = (1 << 6),
    STATUS2_ALS_DIG_SAT     = (1 << 4),
    STATUS2_FD_DIG_SAT      = (1 << 3),
    STATUS2_MOD2_ANA_SAT    = (1 << 2),
    STATUS2_MOD1_ANA_SAT    = (1 << 1),
    STATUS2_MOD0_ANA_SAT    = (1 << 0),
};

enum status3_mask
{
    STATUS3_HYST_STATE_VALID    = (1 << 7),
    STATUS3_HYST_STATE_RD       = (1 << 6),
    STATUS3_AIHT                = (1 << 5),
    STATUS3_AILT                = (1 << 4),
    STATUS3_VSYNC_LOST          = (1 << 3),
    STATUS3_OSC_CALIB_SAT       = (1 << 1),
    STATUS3_OSC_CALIB_FINISHED  = (1 << 0),
};

/* status4 0x9E */
enum status4_mask
{
    STATUS4_MOD_SAMPLE_TRIG_ERR  = (1 << 3),   /* measured data is corrupt    */
    STATUS4_MOD_TRIG_ERR         = (1 << 2),   /* wtime too short             */
    STATUS4_SAI_ACTIVE           = (1 << 1),   /* sleep after int             */
    STATUS4_INIT_BUSY            = (1 << 0),   /* device is init'ing          */
};
#define STATUS4_SAI_ACTIVE_SHIFT    (1)

/* status5 - A0 */
enum status5_mask
{
    STATUS5_SINT_MEAS_SEQR   = (1 << 1),
    STATUS5_SINT_VSYNC       = (1 << 0),
};

/* intenab - 0xBA */
enum intenab_mask
{
    INTENAB_MIEN_MASK          = (1 << 7),
    INTENAB_AIEN_MASK          = (1 << 3),
    INTENAB_FIEN_MASK          = (1 << 2),
    INTENAB_SIEN_MASK          = (1 << 0),
};

void sensor_process_irq(uint8_t *shadow_regs);

#endif /* __TCS3410_IRQ_H__ */

