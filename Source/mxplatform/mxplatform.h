/**
  ******************************************************************************
  * @file           : mxplatform.h
  * @brief          : Header for mxplatform.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MX_PLATFORM_H
#define __MX_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "stm32u5xx_hal.h"
#include "stm32u5x9j_discovery.h"
#include "stm32u5x9j_discovery_hspi.h"
#include "stm32u5x9j_discovery_ospi.h"
#include "gd25lq128e.h"
#include "gpio_config.h"
#include "uart_config.h"
#include "spi_config.h"
#include "tim_config.h"
#include "i2c_config.h"
#include "lcd_config.h"
#include "adc_config.h"
#include "exti_config.h"
#include "sdio_config.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_freertos.h"
#include "jpeg_utils_conf.h"
#include "stm32_lcd.h"
#include "linked_list.h"
#include "image_320x240_argb8888.h"
#include "life_augmented_argb8888.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported variables --------------------------------------------------------*/
extern DMA_HandleTypeDef   handle_GPDMA1_Channel1;
extern DMA_HandleTypeDef   handle_GPDMA1_Channel0;
extern DMA_HandleTypeDef   hdma_adc1;
extern DMA_HandleTypeDef   hdma_adc4;
extern DMA2D_HandleTypeDef hdma2d;
extern GPU2D_HandleTypeDef hgpu2d;
extern JPEG_HandleTypeDef  hjpeg;
extern LTDC_HandleTypeDef  hltdc;

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
void Error_Handler(void);
void MX_PLATFORM_Init(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
#define CF0F_PINMUX_ENABLED      1
#define BRIGHTNESS_USE_LM3597    1

/* MCU_BT_UART2 */
#define BT_UART2_CTS_Pin         GPIO_PIN_0
#define BT_UART2_RTS_Pin         GPIO_PIN_1
#define BT_UART2_TX_Pin          GPIO_PIN_2
#define BT_UART2_RX_Pin          GPIO_PIN_3
#define BT_UART2_GPIO_Port       GPIOA

/* MCU_IO_PA4: GPIO_Analog */

/* SSR_Y_MCU_INT_L: WKUP6 */
#define SSR_INT_Pin              GPIO_PIN_5
#define SSR_INT_GPIO_Port        GPIOA

/* MCU_IO_PA6: GPIO_Analog */

/* ANA_PMIC_VREF_1V2: "ADC1_IN12 / ADC2_IN12 / ADC4_IN20" */
#define PMIC_VREF_Pin            GPIO_PIN_7
#define PMIC_VREF_GPIO_Port      GPIOA

/* MCU_CID0: GPIO_Input */
#define CID0_Pin                 GPIO_PIN_8
#define CID0_GPIO_Port           GPIOA

/* MCU_DBG_UART1_TX / MCU_DBG_UART1_RX */
#define DBG_UART1_TX_Pin         GPIO_PIN_9
#define DBG_UART1_RX_Pin         GPIO_PIN_10
#define DBG_UART1_GPIO_Port      GPIOA

/* MCU_IO_PA11_USB_DM */
/* MCU_IO_PA12_USB_DP */
/* MCU_IO_PA13_SWD_DIO */
/* MCU_IO_PA14_SWD_CLK */
/* MCU_IO_PA15_JTDI */

/* MCU_PMIC_GPIO3: GPIO_Input */
#define PMIC_GPIO3_Pin           GPIO_PIN_0
#define PMIC_GPIO3_GPIO_Port     GPIOB

/* MCU_IO_PB1: GPIO_Analog */
/* MCU_IO_PB2: GPIO_Analog */
/* MCU_IO_PB3_SWD_SWO */
/* MCU_IO_PB4_SWD_NJTRST */
/* MCU_IO_PB5: GPIO_Analog WKUP6 */

/* MCU_MLB_I2C4_SCL: PB6 (defined in stm32u5x9j_discovery_bus.h)*/
/* MCU_MLB_I2C4_SDA: PB7 (defined in stm32u5x9j_discovery_bus.h)*/

/* RADAR_MCU_INT_L: GPIO_EXTI8 WKUP5 */
#define RADAR_INT_Pin            GPIO_PIN_8
#define RADAR_INT_GPIO_Port      GPIOB

/* MCU_DBG_GPIO2: GPIO_Output */
#define DBG_GPIO2_Pin            GPIO_PIN_9

/* MCU_IO_PB10: GPIO_Analog */
#define PP3300_DISP_IO_SW_EN_Pin GPIO_PIN_10
#define PP3300_DISP_GPIO_Port    GPIOB

/* MCU_DATA_FLASH_SPI_CS_L:  OCTOSPIM_ P1_NCS*/
#define DATA_FLASH_SPI_CS_Pin    GPIO_PIN_11
#define DATA_FLASH_CS_GPIO_Port  GPIOB

/* MCU_DBG_GPIO1: GPIO_Output */
#define DBG_GPIO1_Pin            GPIO_PIN_12
#define DBG_GPIO_Port            GPIOB

/* MCU_MLB_I2C2_SCL: PB13 (defined in stm32u5x9j_discovery_bus.h)*/
/* MCU_MLB_I2C2_SDA: PB14 (defined in stm32u5x9j_discovery_bus.h)*/

/* BP_DETECT: GPIO_EXTI15 WKUP7*/
#define BP_DETECT_Pin            GPIO_PIN_15
#define BP_DETECT_GPIO_Port      GPIOB

/* MCU_IO_PC0: GPIO_Analog */
/* MCU_IO_PC1: GPIO_Analog */

/* ANA_WHEEL_SENSOR_VOUT2: "ADC1_IN3/ADC2_IN3/ADC4_IN3" */
#define WHEEL_VOUT2_Pin          GPIO_PIN_2

/* ANA_WHEEL_SENSOR_VOUT1: "ADC1_IN4/ADC2_IN4/ADC4_IN4" */
#define WHEEL_VOUT1_Pin          GPIO_PIN_3
#define WHEEL_VOUT_GPIO_Port     GPIOC

/* MCU_BT_REG_ON: GPIO_Output */
#define BT_REG_ON_Pin            GPIO_PIN_4
#define BT_REG_ON_GPIO_Port      GPIOC

/* DISP_MCU_NTC_READOUT: GPIO_Analog */
#define DISP_NTC_Pin             GPIO_PIN_5
#define DISP_NTC_GPIO_Port       GPIOC

/* MCU_PWM_HVAC_PWR_LOAD_EN: GPIO_Output PWM TIM3_CH1*/
#define HVAC_PWR_LOAD_Pin        GPIO_PIN_6
#define HVAC_PWR_LOAD_GPIO_Port  GPIOC

/* MCU_WIFI_REG_ON: GPIO_Output */
#define WIFI_REG_ON_Pin          GPIO_PIN_7
#define WIFI_REG_ON_GPIO_Port    GPIOC

/* MCU_WIFI_SDIO1 */
#define WIFI_SDIO_D0_Pin         GPIO_PIN_8
#define WIFI_SDIO_D1_Pin         GPIO_PIN_9
#define WIFI_SDIO_D2_Pin         GPIO_PIN_10
#define WIFI_SDIO_D3_Pin         GPIO_PIN_11
#define WIFI_SDIO_CLK_Pin        GPIO_PIN_12
#define WIFI_SDIO_GPIO_Port      GPIOC

/* PMIC_MCU_INT_L */
#define PMIC_INT_Pin             GPIO_PIN_13
#define PMIC_INT_GPIO_Port       GPIOC

/* MCU_IO_PC14_CLK_32K_IN */
/* MCU_IO_PC15_CLK_32K_OUT */

/* MCU_IO_PD0: GPIO_Analog */

/* MCU_CID1 */
#define CID1_Pin                 GPIO_PIN_1
#define CID1_GPIO_Port           GPIOD

/* MCU_WIFI_SDIO1_CMD */
#define WIFI_SDIO1_CMD_Pin       GPIO_PIN_2
#define WIFI_SDIO_CMD_GPIO_Port  GPIOD

/* MCU_DISP_CP_ENN: GPIO_Output */
#define DISP_CP_ENN_Pin          GPIO_PIN_3

/* DISP_MCU_ID0: GPIO_Input */
#define DISP_ID0_Pin             GPIO_PIN_4

/* MCU_IO_PD5: GPIO_Analog */
/* MCU_IO_PD6: GPIO_Analog */

/* MCU_RADAR_PP1V8_LDO_EN: GPIO_Output */
#define RADAR_PP1V8_LDO_EN_Pin   GPIO_PIN_7

/* MCU_DISP_CP_ENP: GPIO_Output */
#define DISP_CP_ENP_Pin          GPIO_PIN_8

/* DISP_MCU_ID1: GPIO_Input */
#define DISP_ID1_Pin             GPIO_PIN_9
#define DISP_PORTD_GPIO_Port     GPIOD

/* MCU_RADAR_RST_L: GPIO_Output */
#define RADAR_RST_Pin            GPIO_PIN_10

/* ANA_USB1_VBUS_SENSE: ADC4_IN15 */
#define ANA_USB1_VBUS_Pin        GPIO_PIN_11

/* ANA_VBAT_MEAS: ADC4_IN16 */
#define ANA_VBAT_MEAS_Pin        GPIO_PIN_12

/* ANA_BP_MUX_OUT: ADC4_IN17 */
#define ANA_BP_MUX_OUT_Pin       GPIO_PIN_13
#define ANA_ADC4_GPIO_Port       GPIOD

/* MCU_BT_DEV_WAKE: GOIP_Output WAKEUP */
#define BT_DEV_WAKE_Pin          GPIO_PIN_14
#define BT_DEV_WAKE_GPIO_Port    GPIOD

/* MCU_RADAR_OSC_EN: GPIO_Output */
#define RADAR_OSC_EN_Pin         GPIO_PIN_15
#define RADAR_PORTD_GPIO_Port    GPIOD

/* MCU_BID0: PE0 GPIO_Input */
#define BID0_Pin                 GPIO_PIN_0
#define BID0_GPIO_Port           GPIOE

/* DISP_MCU_TE_INT_EDGE: PE1 GPIO_Input */
#define DISP_TE_INT_Pin          GPIO_PIN_1

/* MCU_IO_PE2: GPIO_Analog */

/* MCU_ADC_THERM_SOU_EST_EN: GPIO_Output */
#define THERM_SOU_EST_EN_Pin     GPIO_PIN_3
#define THERM_SOU_EST_GPIO_Port  GPIOE

/* PMIC_MCU_UX_BUTTON_L: GPIO_EXTI4 INT WKUP1 */
#define PMIC_UX_BUTTON_Pin       GPIO_PIN_4
#define PMIC_UX_BUTTON_GPIO_Port GPIOE

/* MCU_DISP_BL_EN: GPIO_Output */
#define DISP_BL_EN_Pin           GPIO_PIN_5

/* BT_MCU_HOST_WAKE: GPIO_EXTI6 INT WKUP3 */
#define BT_HOST_WAKE_Pin         GPIO_PIN_6
#define BT_HOST_WAKE_GPIO_Port   GPIOE

/* MCU_IO_PE7: GPIO_Analog */

/* MCU_PWM_PIEZO_N_1V8: TIM1_CH1N PWM */
#define PWM_PIEZO_N_Pin          GPIO_PIN_8

/* MCU_PWM_PIEZO_P_1V8: TIM1_CH1 PWM */
#define PWM_PIEZO_P_Pin          GPIO_PIN_9
#define PWM_PIEZO_GPIO_Port      GPIOE

/* MCU_IO_PE10: GPIO_Analog */

/* MCU_DISP_RST_L: GPIO_Output */
#define DISP_RST_Pin             GPIO_PIN_11

/* MCU_PP1800_DISP_IO_SW_EN: GPIO_Output */
#define PP1800_DISP_IO_SW_EN_Pin  GPIO_PIN_12
#define DISP_PORTE_GPIO_Port      GPIOE

/* MCU_SAPS_CTRL_EN: GPIO_Output */
#define SAPS_CTRL_EN_Pin         GPIO_PIN_13

/* MCU_SAPS_PHASE_DELAY_SEL: GPIO_Output */
#define SAPS_PHASE_DELAY_SEL_Pin GPIO_PIN_14

/* MCU_SAPS_PULSE_WIDTH_SEL: GPIO_Output */
#define SAPS_PULSE_WIDTH_SEL_Pin  GPIO_PIN_15
#define SAPS_GPIO_Port            GPIOE

/* MCU_BP_SENS_I2C6_SDA: PF0 (defined in stm32u5x9j_discovery_bus.h)*/
/* MCU_BP_SENS_I2C6_SCL: PF1 (defined in stm32u5x9j_discovery_bus.h)*/

/* WIFI_MCU_HOST_WAKE: GPIO_EXTI2 INT WKUP8 */
#define WIFI_HOST_WAKE_Pin        GPIO_PIN_2
#define WIFI_HOST_WAKE_GPIO_Port  GPIOF

/* MCU_BP_RS485_PHY_UART5_TX: PF3 */
#define RS485_UART5_TX_Pin        GPIO_PIN_3
/* MCU_BP_RS485_PHY_UART5_RX: PF4 */
#define RS485_UART5_RX_Pin        GPIO_PIN_4
#define RS485_UART5_GPIO_Port     GPIOF

/* MCU_IO_PF5: GPIO_Analog */

/* MCU_DATA_FLASH_SPI: OCTOSPIM_ P1_IO */
#define DATA_FLASH_SPI_D3_Pin     GPIO_PIN_6
#define DATA_FLASH_SPI_D2_Pin     GPIO_PIN_7
#define DATA_FLASH_SPI_D1_Pin     GPIO_PIN_9
#define DATA_FLASH_SPI_D0_Pin     GPIO_PIN_8
#define DATA_FLASH_SPI_CLK_Pin    GPIO_PIN_10
#define DATA_FLASH_SPI_GPIO_Port  GPIOF

/* PWR_MON_HVAC_MCU_ALERT_L: GPIO_EXTI11 INT */
#define PWR_MON_HVAC_ALERT_Pin    GPIO_PIN_11
#define MON_HVAC_ALERT_GPIO_Port  GPIOF

/* PWR_MON_SYS_MCU_ALERT_L: GPIO_EXTI12 INT */
#define PWR_MON_SYS_ALERT_Pin     GPIO_PIN_12
#define MON_SYS_ALERT_GPIO_Port   GPIOF

/* MCU_WIFI_DEV_WAKE: GPIO_Output */
#define WIFI_DEV_WAKE_Pin         GPIO_PIN_13
#define WIFI_DEV_WAKE_GPIO_Port   GPIOF

/* MCU_IO_PF14: GPIO_Analog */

/* ANA_DCIN1_MEAS: ADC4_IN6 */
#define ANA_DCIN1_MEAS_Pin        GPIO_PIN_15
#define ANA_DCIN1_MEAS_GPIO_Port  GPIOF

/* ANA_THERM_NTC: ADC4_IN7 */
#define ANA_THERM_NTC_Pin         GPIO_PIN_0
#define ANA_THERM_NTC_GPIO_Port   GPIOG

/* ANA_HW_ID: ADC4_IN8 */
#define ANA_HW_ID_Pin             GPIO_PIN_1
#define ANA_HW_ID_GPIO_Port       GPIOG

/* MCU_IO_PG2: GPIO_Analog */
/* MCU_IO_PG3: GPIO_Analog */

/* MCU_ADC_HW_ID_EN: GPIO_Output */
#define ADC_HW_ID_EN_Pin          GPIO_PIN_4
#define ADC_HW_ID_GPIO_Port       GPIOG

/* MCU_IO_PG5: GPIO_Analog */
/* MCU_IO_PG6: GPIO_Analog */
/* MCU_IO_PG7: GPIO_Analog */
/* MCU_IO_PG8: GPIO_Analog */

/* MCU_RADAR_PP1V2_LDO_EN */
#define RADAR_PP1V2_LDO_EN_Pin    GPIO_PIN_9
#define RADAR_PORTG_GPIO_Port     GPIOG

/* MCU_IO_PG10: GPIO_Analog */

/* MCU_WHEEL_SENSOR_EN: GPIO_Output */
#define WHEEL_SENSOR_EN_Pin       GPIO_PIN_11
#define WHEEL_SENSOR_GPIO_Port    GPIOG

/* MCU_BID1: GPIO_Input */
#define BID1_Pin                  GPIO_PIN_12
#define BID1_GPIO_Port            GPIOG

/* MCU_BP_SSR_I2C1_SDA: PG13 (defined in stm32u5x9j_discovery_bus.h)*/
/* MCU_BP_SSR_I2C1_SCL: PG14 (defined in stm32u5x9j_discovery_bus.h)*/

/* MCU_IO_PG15: GPIO_Analog */

/* MCU_IO_PH0_XTALI_16MHZ: PH0 */
/* MCU_IO_PH1_XTALO_16MHZ: PH1 */

/* MCU_CID2: GPIO_Input */
#define CID2_Pin                  GPIO_PIN_2
#define CID2_GPIO_Port            GPIOH

/* MCU_BOOT0: PH3 GPIO_Input BOOT */
/* MCU_IO_PH4: GPIO_Analog */
/* MCU_IO_PH5: GPIO_Analog */
/* MCU_IO_PH6: GPIO_Analog */
/* MCU_IO_PH7: GPIO_Analog */
/* MCU_IO_PH8: GPIO_Analog */
/* MCU_IO_PH9: GPIO_Analog */

/* MCU_ADC_THERM_SOU_WST_EN: GPIO_Output */
#define THERM_SOU_WST_EN_Pin      GPIO_PIN_10
#define THERM_SOU_WST_GPIO_Port   GPIOH

/* DISP_MCU_ID2: GPIO_Input */
#define DISP_ID2_Pin              GPIO_PIN_11
#define DISP_ID2_GPIO_Port        GPIOH

/* MCU_IO_PH12: GPIO_Analog */
/* MCU_IO_PH13: GPIO_Analog */

/* PMIC_MCU_PGOOD: GPIO_EXTI11 INT */
#define PMIC_PGOOD_Pin            GPIO_PIN_14
#define PMIC_PGOOD_GPIO_Port      GPIOH

/* MCU_IO_PH15: GPIO_Analog */

/* MCU_RADAR_SPI2 */
#define RADAR_SPI2_CS_Pin         GPIO_PIN_0
#define RADAR_SPI2_CLK_Pin        GPIO_PIN_1
#define RADAR_SPI2_MISO_Pin       GPIO_PIN_2
#define RADAR_SPI2_MOSI_Pin       GPIO_PIN_3
#define RADAR_SPI2_GPIO_Port      GPIOI

/* MCU_LED_AMBER_EN_L */
#define LED_AMBER_EN_Pin          GPIO_PIN_4
#define LED_AMBER_EN_GPIO_Port    GPIOI

/* MCU_BID2 */
#define BID2_Pin                  GPIO_PIN_5
#define BID2_GPIO_Port            GPIOI

/* MCU_ADC_THERM_NOR_WST_EN: GPIO_Output */
#define THERM_NOR_WST_EN_Pin      GPIO_PIN_6
#define THERM_NOR_WST_GPIO_Port   GPIOI

/* MCU_IO_PI7: GPIO_Analog */
/* MCU_IO_PI8: GPIO_Analog */

/* BP_IOEXP_MCU_INT_L: GPIO_EXTI9 INT */
#define BP_IOEXP_INT_Pin          GPIO_PIN_9
#define BP_IOEXP_INT_GPIO_Port    GPIOI

/* MCU_PMIC_RST_L: GPIO_Output */
#define PMIC_RST_Pin              GPIO_PIN_10
#define PMIC_RST_GPIO_Port        GPIOI

/* MCU_IO_PI11: GPIO_Analog */

/* MCU_VBAT_MEAS_EN: GPIO_Output */
#define VBAT_MEAS_EN_Pin          GPIO_PIN_12
#define VBAT_MEAS_EN_GPIO_Port    GPIOI

/* MCU_IO_PI13: GPIO_Analog */
/* MCU_IO_PI14: GPIO_Analog */
/* MCU_IO_PI15: GPIO_Analog */

/* TOUCH_MCU_INT_L: GPIO_EXTI0 INT */
#define TOUCH_INT_Pin             GPIO_PIN_0
#define TOUCH_INT_GPIO_Port       GPIOJ

/* AMS_INT: GPIO_EXTI4 */
#define AMS_INT_Pin               GPIO_PIN_4
#define AMS_INT_GPIO_Port         GPIOC
#define AMS_INT_EXTI_IRQn         EXTI4_IRQn

#define TOF_LPN_Pin               GPIO_PIN_14
#define TOF_LPN_GPIO_Port         GPIOE

#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOE
#define LED_RED_Pin GPIO_PIN_1
#define LED_RED_GPIO_Port GPIOE
#define LCD_RESET_Pin GPIO_PIN_5
#define LCD_RESET_GPIO_Port GPIOD
#define DSI_PWR_ON_Pin GPIO_PIN_5
#define DSI_PWR_ON_GPIO_Port GPIOI
#define VSYNC_FREQ_Pin GPIO_PIN_1
#define VSYNC_FREQ_GPIO_Port GPIOD
#define DSI_BL_CTRL_Pin GPIO_PIN_6
#define DSI_BL_CTRL_GPIO_Port GPIOI
#define RENDER_TIME_Pin GPIO_PIN_0
#define RENDER_TIME_GPIO_Port GPIOD
#define USER_BUTTON_Pin GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define OSPI_CLK_Pin GPIO_PIN_10
#define OSPI_CLK_GPIO_Port GPIOF
#define OSPI_D2_Pin GPIO_PIN_7
#define OSPI_D2_GPIO_Port GPIOF
#define OSPI_D1_Pin GPIO_PIN_9
#define OSPI_D1_GPIO_Port GPIOF
#define OSPI_D4_Pin GPIO_PIN_1
#define OSPI_D4_GPIO_Port GPIOC
#define OSPI_D3_Pin GPIO_PIN_6
#define OSPI_D3_GPIO_Port GPIOF
#define OSPI_D0_Pin GPIO_PIN_8
#define OSPI_D0_GPIO_Port GPIOF
#define OSPI_CS_Pin GPIO_PIN_2
#define OSPI_CS_GPIO_Port GPIOA
#define DSI_TOUCH_INT_Pin GPIO_PIN_8
#define DSI_TOUCH_INT_GPIO_Port GPIOE
#define DSI_TOUCH_INT_EXTI_IRQn EXTI8_IRQn
#define OSPI_D6_Pin GPIO_PIN_3
#define OSPI_D6_GPIO_Port GPIOC
#define MCU_ACTIVE_Pin GPIO_PIN_12
#define MCU_ACTIVE_GPIO_Port GPIOF
#define OSPI_D5_Pin GPIO_PIN_2
#define OSPI_D5_GPIO_Port GPIOC
#define OSPI_D7_Pin GPIO_PIN_0
#define OSPI_D7_GPIO_Port GPIOC
#define OSPI_DQS_Pin GPIO_PIN_1
#define OSPI_DQS_GPIO_Port GPIOA
#define FRAME_RATE_Pin GPIO_PIN_14
#define FRAME_RATE_GPIO_Port GPIOF

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MX_PLATFORM_H */
