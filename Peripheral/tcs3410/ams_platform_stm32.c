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
#include "ams_device.h"
#include "ams_errno.h"
#include "ams_platform.h"
#include "stm32u5x9j_discovery_bus.h"
#include "stm32u5x9j_discovery_errno.h"

/*
 * Not currently used by tcs3410 - But make sure they are init'd off.
 */

static ams_errno_t ams_stm32_i2c_init(uint8_t scl_pin, uint8_t sda_pin)
{

    return BSP_I2C1_Init();
}

static void ams_stm32_platform_log(int level, const char *pLogMsg)
{
/* Single log level for easier testing - no escape sequence characters for color */
#if defined(USE_SINGLE_LOG_LEVEL)
    level = LOG_INFO;
#endif

    switch (level)
    {
        case LOG_INFO:
        {
            //NRF_LOG_INFO("%s", nrf_log_push((char *)pLogMsg));
            break;
        }
        case LOG_WARNING:
        {
            //NRF_LOG_WARNING("%s", nrf_log_push((char *)pLogMsg));
            break;
        }
        case LOG_DEBUG: /* Currently this is Nordic broke */
        {
            //NRF_LOG_DEBUG("%s", nrf_log_push((char *)pLogMsg));
            break;
        }
        case LOG_ERROR:
        {
            //NRF_LOG_ERROR("%s", nrf_log_push((char *)pLogMsg));
            break;
        }
    }
    //NRF_LOG_FLUSH();
    return;
}

static void ams_stm32_irq_init(uint32_t irq_pin)
{

    return;
}

/* All platforms must define this function */
void ams_platform_init(struct ams_platform *pPlatform)
{
    pPlatform->ams_platform_i2c_init   = ams_stm32_i2c_init;
    pPlatform->ams_platform_log        = ams_stm32_platform_log;
    pPlatform->ams_platform_irq_init   = ams_stm32_irq_init;
    pPlatform->ams_platform_spi_init   = NULL;
    pPlatform->ams_platform_gpio_init  = NULL;
    pPlatform->ams_platform_timer_init = NULL;

    return;
}

