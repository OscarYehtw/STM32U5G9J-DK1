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

#ifndef __AMS_PLATFORM_H__
#define __AMS_PLATFORM_H__

#define AMS_SCL_PIN                    (27)
#define AMS_SDA_PIN                    (26)
#define AMS_IRQ0_PIN                   (14)  /* Maps to BSP_BUTTON_0 which maps to Button_1 - see boards.h */
#define AMS_IRQ1_PIN                   (15)

/*
 * Used for debugging on the Nordic M4 MCU board - logging
 * from IRQ is not efficient or 100% accurate.
 */
#define EVENT0_LED                         (0)
#define EVENT1_LED                         (1)

#define EVENT2_LED                         (2)
#define EVENT3_LED                         (3)

#define STM32_LOG_INFO(...)  platform_log_output(__VA_ARGS__)
#define STM32_LOG_WARNING(...)  platform_log_output(__VA_ARGS__)
#define STM32_LOG_DEBUG(...)  platform_log_output(__VA_ARGS__)
#define STM32_LOG_ERROR(...) platform_log_output(__VA_ARGS__)

/* Platform Specific Definitions */
typedef ams_errno_t (*platform_i2c_init)(uint8_t scl_pin, uint8_t sda_pin);
typedef void (*platform_log)(int level, const char *pLogMsg);
typedef void (*platform_irq_event_handler)(void *data, uint16_t size);
typedef void (*platform_irq_init)(uint32_t pin);
typedef void (*platform_spi_init)(void);
typedef void (*platform_gpio_init)(void);
typedef void (*platform_timer_init)(void);

struct ams_platform
{
    platform_i2c_init    ams_platform_i2c_init;
    platform_log         ams_platform_log;
    platform_irq_init    ams_platform_irq_init;
    platform_spi_init    ams_platform_spi_init;
    platform_gpio_init   ams_platform_gpio_init;
    platform_timer_init  ams_platform_timer_init;
};


/* ams_device_init() calls this for all platforms */
void ams_platform_init(struct ams_platform *pPlatform);
ams_errno_t ams_convert_platform_error_code(int error_code);
void ams_platform_LED_invert(uint32_t idx);
void ams_platform_LED_off(uint32_t idx);
void ams_platform_LED_on(uint32_t idx);

#endif   /* __AMS_PLATFORM_H__ */