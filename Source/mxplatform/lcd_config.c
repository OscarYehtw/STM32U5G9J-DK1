/**
  ******************************************************************************
  * @file    lcd_config.c
  * @brief   LCD Configuration.
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

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mxplatform.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
DMA2D_HandleTypeDef  hdma2d;
GPU2D_HandleTypeDef  hgpu2d;
DSI_HandleTypeDef    hdsi;
LTDC_HandleTypeDef   hltdc;
JPEG_HandleTypeDef   hjpeg;

LCD_UTILS_Drv_t      LCD_Driver;
char str_display[60] = "";
static uint32_t ImageIndex = 0;
static const uint32_t * Images[] =
{
  image_320x240_argb8888,
  life_augmented_argb8888,
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* External function --------------------------------------------------------*/

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  Configure the default LCD clock and perform panel reset.
  * @note   This function switches the DSI interface to use the DSI PHY PLL
  *         as the clock source, ensuring stable timing for LCD operation.
  *         After configuring the clock, it issues a hardware reset to the LCD
  *         panel by toggling the reset GPIO pin with proper delays.
  * @param  None
  * @retval None
  */
void LCD_Set_Default_Clock(void)
{
  RCC_PeriphCLKInitTypeDef  DSIPHYInitPeriph;

  /* Switch to DSI PHY PLL clock */
  DSIPHYInitPeriph.PeriphClockSelection = RCC_PERIPHCLK_DSI;
  DSIPHYInitPeriph.DsiClockSelection    = RCC_DSICLKSOURCE_DSIPHY;

  HAL_RCCEx_PeriphCLKConfig(&DSIPHYInitPeriph);

#if 0
  /* LCD Reset */
  HAL_Delay(11);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_Delay(150);
#endif

}

/**
  * @brief  Converts a line to an ARGB8888 pixel format.
  * @param  pSrc: Pointer to source buffer
  * @param  pDst: Output color
  * @param  xSize: Buffer width
  * @param  ColorMode: Input color mode
  * @retval None
  */
static void CopyBuffer(uint32_t *pSrc, uint32_t *pDst, uint16_t x, uint16_t y, uint16_t xsize, uint16_t ysize)
{

  uint32_t destination = (uint32_t)pDst + (y * 480 + x) * 4;
  uint32_t source      = (uint32_t)pSrc;

  hdma2d.Init.OutputOffset = 480 - xsize;


  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
  {
      if (HAL_DMA2D_Start(&hdma2d, source, destination, xsize, ysize) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d, 100);
    }
  }
}

/**
  * @brief  Check for user input.
  * @param  None
  * @retval Input state (1 : active / 0 : Inactive)
  */
uint32_t SetPanelConfig(void)
{
  if(HAL_DSI_Start(&hdsi) != HAL_OK) return 1;

  /* CMD Mode */
  uint8_t InitParam1[3] = {0xFF ,0x83 , 0x79};
  if (HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 3, 0xB9, InitParam1) != HAL_OK) return 1;

  /* SETPOWER */
  uint8_t InitParam3[16] = {0x44,0x1C,0x1C,0x37,0x57,0x90,0xD0,0xE2,0x58,0x80,0x38,0x38,0xF8,0x33,0x34,0x42};
  if (HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 16, 0xB1, InitParam3) != HAL_OK) return 2;

  /* SETDISP */
  uint8_t InitParam4[9] = {0x80,0x14,0x0C,0x30,0x20,0x50,0x11,0x42,0x1D};
  if (HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 9, 0xB2, InitParam4) != HAL_OK) return 3;

  /* Set display cycle timing */
  uint8_t InitParam5[10] = {0x01,0xAA,0x01,0xAF,0x01,0xAF,0x10,0xEA,0x1C,0xEA};
  if (HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 10, 0xB4, InitParam5) != HAL_OK) return 4;

  /* SETVCOM */
  uint8_t InitParam60[4] = {00,00,00,0xC0};
  if (HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 4, 0xC7, InitParam60) != HAL_OK) return 5;

  /* Set Panel Related Registers */
  if (HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xCC, 0x02) != HAL_OK) return 6;

  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xD2, 0x77) != HAL_OK) return 7;

  uint8_t InitParam50[37] = {0x00,0x07,0x00,0x00,0x00,0x08,0x08,0x32,0x10,0x01,0x00,0x01,0x03,0x72,0x03,0x72,0x00,0x08,0x00,0x08,0x33,0x33,0x05,0x05,0x37,0x05,0x05,0x37,0x0A,0x00,0x00,0x00,0x0A,0x00,0x01,0x00,0x0E};
  if (HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 37, 0xD3, InitParam50) != HAL_OK) return 8;

  uint8_t InitParam51[34] = {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x19,0x19,0x18,0x18,0x18,0x18,0x19,0x19,0x01,0x00,0x03,0x02,0x05,0x04,0x07,0x06,0x23,0x22,0x21,0x20,0x18,0x18,0x18,0x18,0x00,0x00};
  if (HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 34, 0xD5, InitParam51) != HAL_OK) return 9;

  uint8_t InitParam52[35] = {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x19,0x19,0x18,0x18,0x19,0x19,0x18,0x18,0x06,0x07,0x04,0x05,0x02,0x03,0x00,0x01,0x20,0x21,0x22,0x23,0x18,0x18,0x18,0x18};
  if (HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 35, 0xD6, InitParam52) != HAL_OK) return 10;

  /* SET GAMMA */
  uint8_t InitParam8[42] = {0x00,0x16,0x1B,0x30,0x36,0x3F,0x24,0x40,0x09,0x0D,0x0F,0x18,0x0E,0x11,0x12,0x11,0x14,0x07,0x12,0x13,0x18,0x00,0x17,0x1C,0x30,0x36,0x3F,0x24,0x40,0x09,0x0C,0x0F,0x18,0x0E,0x11,0x14,0x11,0x12,0x07,0x12,0x14,0x18};
  if (HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 42, 0xE0, InitParam8) != HAL_OK) return 11;

  uint8_t InitParam44[3] = {0x2C,0x2C,00};
  if (HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 3, 0xB6, InitParam44) != HAL_OK) return 12;

  if (HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xBD, 0x00) != HAL_OK) return 13;

  uint8_t InitParam14[] = {0x01,0x00,0x07,0x0F,0x16,0x1F,0x27,0x30,0x38,0x40,0x47,0x4E,0x56,0x5D,0x65,0x6D,0x74,0x7D,0x84,0x8A,0x90,0x99,0xA1,0xA9,0xB0,0xB6,0xBD,0xC4,0xCD,0xD4,0xDD,0xE5,0xEC,0xF3,0x36,0x07,0x1C,0xC0,0x1B,0x01,0xF1,0x34,0x00};
  if (HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 42, 0xC1, InitParam14) != HAL_OK) return 14;

  if (HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xBD, 0x01) != HAL_OK) return 15;

  uint8_t InitParam15[] = {0x00,0x08,0x0F,0x16,0x1F,0x28,0x31,0x39,0x41,0x48,0x51,0x59,0x60,0x68,0x70,0x78,0x7F,0x87,0x8D,0x94,0x9C,0xA3,0xAB,0xB3,0xB9,0xC1,0xC8,0xD0,0xD8,0xE0,0xE8,0xEE,0xF5,0x3B,0x1A,0xB6,0xA0,0x07,0x45,0xC5,0x37,0x00};
  if (HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 42, 0xC1, InitParam15) != HAL_OK) return 16;

  if (HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xBD, 0x02) != HAL_OK) return 17;

  uint8_t InitParam20[42] = {0x00,0x09,0x0F,0x18,0x21,0x2A,0x34,0x3C,0x45,0x4C,0x56,0x5E,0x66,0x6E,0x76,0x7E,0x87,0x8E,0x95,0x9D,0xA6,0xAF,0xB7,0xBD,0xC5,0xCE,0xD5,0xDF,0xE7,0xEE,0xF4,0xFA,0xFF,0x0C,0x31,0x83,0x3C,0x5B,0x56,0x1E,0x5A,0xFF};
  if (HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, 42, 0xC1, InitParam20) != HAL_OK) return 18;

  if (HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xBD, 0x00) != HAL_OK) return 19;

  /* Exit Sleep Mode*/
  if (HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P0, 0x11, 0x00) != HAL_OK) return 20;

  HAL_Delay(120);

  /* Clear LCD_FRAME_BUFFER */
  memset((uint32_t *)LCD_FRAME_BUFFER,0x00, 0xFFFFF);

  /* Display On */
  if (HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P0, 0x29, 0x00) != HAL_OK) return 21;

  HAL_Delay(120);

  /* All setting OK */
  return 0;
}

/**
  * @brief  Configure the ST7701S MIPI-DSI Display Panel.
  *
  *         This function sends the full initialization command sequence to
  *         the ST7701S LCD driver over the MIPI DSI interface. It includes:
  *           - Sleep Out
  *           - CMD2 page switching
  *           - Power voltage settings
  *           - Gamma correction tables
  *           - GIP (Gate driver timing)
  *           - Color control, sharpness, noise reduction
  *           - Display ON command
  *
  *         All commands are sent using HAL_DSI_ShortWrite() or
  *         HAL_DSI_LongWrite(), depending on the payload length.
  *
  * @param  None
  * @retval uint32_t 0 if OK, non-zero value indicates error step
  */
uint32_t SetPanelConfig_ST7701S(void)
{
  if (HAL_DSI_Start(&hdsi) != HAL_OK) return 1;
  HAL_Delay(120);

  /* Sleep Out */
  if (HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P0, 0x11, 0x00) != HAL_OK) return 2;
  HAL_Delay(120);

  /* CMD2 Page 0, 0xFF: System Function Command Table2 */
  uint8_t cmd_ff0[] = {0x77,0x01,0x00,0x00,0x10};
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(cmd_ff0), 0xFF, cmd_ff0) != HAL_OK) return 3;
  HAL_Delay(5);

  /* C0 Display Line Setting ((0x63 + 1) * 8) = 800 */
  uint8_t c0_param[] = {0x63,0x00};
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(c0_param), 0xC0, c0_param) != HAL_OK) return 4;
  HAL_Delay(5);

  /* C1 Porch Control, VBP=17, VFP=02 */
  uint8_t c1_param[] = {0x11,0x02};
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(c1_param), 0xC1, c1_param) != HAL_OK) return 5;
  HAL_Delay(5);

  /* C2 Inversion + Frame rate, 0x31: 0x11: 2 Dot, 0x10: 1 Dot, 0x17: Column, 0x08: Default */
  uint8_t c2_param[] = {0x31,0x08};
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(c2_param), 0xC2, c2_param) != HAL_OK) return 6;

  /* CC NVM Program Active */
  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xCC, 0x10) != HAL_OK) return 7;
  HAL_Delay(5);

  /* B0 Positive Gamma */
  uint8_t gamma_p[] = {
      0x00,0x0E,0x55,0x0B,0x0E,0x05,0x04,0x09,
      0x09,0x20,0x06,0x14,0x12,0xA3,0x29,0x18
  };
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(gamma_p), 0xB0, gamma_p) != HAL_OK) return 8;
  HAL_Delay(5);

  /* B1 Negative Gamma */
  uint8_t gamma_n[] = {
      0x00,0x0E,0x55,0x0B,0x0E,0x04,0x04,0x08,
      0x08,0x20,0x06,0x14,0x12,0xA2,0x28,0x18
  };
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(gamma_n), 0xB1, gamma_n) != HAL_OK) return 9;
  HAL_Delay(5);

  /* CMD2 Page 1 */
  uint8_t cmd_ff1[] = {0x77,0x01,0x00,0x00,0x11};
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(cmd_ff1), 0xFF, cmd_ff1) != HAL_OK) return 10;
  HAL_Delay(5);

  /* B0 Vop Amplitude, 0x81: GVDD=5.15V */
  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB0, 0x81) != HAL_OK) return 11;

  /* B1 VCOM setting, 0x4F: 1.1125 */
  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB1, 0x4F) != HAL_OK) return 12;

  /* B2 VGH = 15V */
  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB2, 0x07) != HAL_OK) return 13;

  /* B3 Gate EQ */
  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB3, 0x80) != HAL_OK) return 14;

  /* B5 VGL = -9.51V */
  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB5, 0x47) != HAL_OK) return 15;

  /* B7 Power Control 1 */
  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB7, 0x85) != HAL_OK) return 16;

  /* B8 AVDD/AVCL, 0x21: 6.6, -4.6 */
  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB8, 0x21) != HAL_OK) return 17;

  /* B9 Power Control 2 */
  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xB9, 0x10) != HAL_OK) return 18;

  /* C1 Source pre-driver timing */
  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xC1, 0x78) != HAL_OK) return 19;

  /* C2 Source EQ2 */
  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xC2, 0x78) != HAL_OK) return 20;

  /* D0 MIPI Setting 1 */
  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0xD0, 0x88) != HAL_OK) return 21;

  HAL_Delay(100);

  /* E0 Sunlight Enhancement */
  uint8_t e0_param[] = {0x00,0x00,0x02};
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(e0_param), 0xE0, e0_param) != HAL_OK) return 22;

  /* E1 Noise Reduce */
  uint8_t e1_param[] = {0x08,0x00,0x0A,0x00,0x07,0x00,0x09,0x00,0x00,0x33,0x33};
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(e1_param), 0xE1, e1_param) != HAL_OK) return 23;

  /* E2 Sharpness */
  uint8_t e2_param[12] = {0};
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(e2_param), 0xE2, e2_param) != HAL_OK) return 24;

  /* E3 Color Calibration Control */
  uint8_t e3_param[] = {0x00,0x00,0x33,0x33};
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(e3_param), 0xE3, e3_param) != HAL_OK) return 25;

  /* E4 Skin Tone Preservation Control */
  uint8_t e4_param[] = {0x44,0x44};
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(e4_param), 0xE4, e4_param) != HAL_OK) return 26;

  /* E5 GIP Setting*/
  uint8_t e5_param[] = {
      0x0E,0x2D,0xA0,0xA0,0x10,0x2D,0xA0,0xA0,
      0x0A,0x2D,0xA0,0xA0,0x0C,0x2D,0xA0,0xA0
  };
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(e5_param), 0xE5, e5_param) != HAL_OK) return 27;

  /* E6 GIP2 */
  uint8_t e6_param[] = {0x00,0x00,0x33,0x33};
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(e6_param), 0xE6, e6_param) != HAL_OK) return 28;

  /* E7 GIP3 */
  uint8_t e7_param[] = {0x44,0x44};
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(e7_param), 0xE7, e7_param) != HAL_OK) return 29;

  /* E8 GIP Setting */
  uint8_t e8_param[] = {
      0x0D,0x2D,0xA0,0xA0,0x0F,0x2D,0xA0,0xA0,
      0x09,0x2D,0xA0,0xA0,0x0B,0x2D,0xA0,0xA0
  };
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(e8_param), 0xE8, e8_param) != HAL_OK) return 30;
  HAL_Delay(5);

  /* EB GIP Setting */
  uint8_t eb_param[] = {
      0x02,0x01,0xE4,0xE4,0x44,0x00,0x40
  };
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(eb_param), 0xEB, eb_param) != HAL_OK) return 31;
  HAL_Delay(5);

  /* EC GIP Setting */
  uint8_t ec_param[] = {0x02,0x01};
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(ec_param), 0xEC, ec_param) != HAL_OK) return 32;
  HAL_Delay(5);

  /* ED GIP Setting */
  uint8_t ed_param[] = {
      0xAB,0x89,0x76,0x54,0x01,0xFF,0xFF,0xFF, 
      0xFF,0xFF,0xFF,0x10,0x45,0x67,0x98,0xBA
  };
  if(HAL_DSI_LongWrite(&hdsi, 0, DSI_DCS_LONG_PKT_WRITE, sizeof(ed_param), 0xED, ed_param) != HAL_OK) return 33;
  HAL_Delay(5);

  /* Display ON */
  if(HAL_DSI_ShortWrite(&hdsi, 0, DSI_DCS_SHORT_PKT_WRITE_P0, 0x29, 0x00) != HAL_OK) return 34;
  HAL_Delay(120);

  return 0;
}

int32_t LCD_FillRGBRect(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint8_t *pData, uint32_t Width, uint32_t Height)
{
  uint32_t Xaddress = 0;
  uint32_t StartAddress = 0;
  uint32_t i;
  uint32_t j;

  /* Set the start address */
  StartAddress = (hltdc.LayerCfg[0].FBStartAdress + (4 * (Ypos * 480 + Xpos)));


  /* Fill the rectangle */
  for (i = 0; i < Height; i++)
  {
    Xaddress = StartAddress + (480 * 4 * i);
    for (j = 0; j < Width; j++)
    {
      *(__IO uint32_t *)(Xaddress) = *(uint32_t *) pData;
      pData += 4;
      Xaddress += 4;
    }
  }

  return 0;
}

int32_t LCD_FillRect(uint32_t Instance, uint32_t Xpos, uint32_t Ypos, uint32_t Width, uint32_t Height, uint32_t Color)
{
  uint32_t  Xaddress = 0;
  uint32_t  Startaddress = 0;
  uint32_t  i;
  uint32_t  j;

  /* Get the rectangle start address */
  Startaddress = (hltdc.LayerCfg[0].FBStartAdress + (4 * (Ypos * 480 + Xpos)));

  /* Fill the rectangle */
  for (i = 0; i < Height; i++)
  {
    Xaddress = Startaddress + (480 * 4 * i);
    for (j = 0; j < Width; j++)
    {
      *(__IO uint32_t *)(Xaddress) = Color;
      Xaddress += 4;
    }
  }

  return 0;
}

int32_t LCD_GetXSize(uint32_t Instance, uint32_t *Xsize)
{
  /* Get the display Xsize */
  *Xsize = 480U;

  return 0;
}

int32_t LCD_GetYSize(uint32_t Instance, uint32_t *Ysize)
{

  /* Get the display Ysize */
  *Ysize = 480U;

  return 0;
}


int32_t LCD_GetFormat(uint32_t Instance, uint32_t *Format)
{
  /* Get pixel format supported by LCD */
  *Format = LTDC_PIXEL_FORMAT_ARGB8888;

  return 0;
}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.Init.BytesSwap = DMA2D_BYTES_REGULAR;
  hdma2d.Init.LineOffsetMode = DMA2D_LOM_PIXELS;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief GPU2D Initialization Function
  * @param None
  * @retval None
  */
void MX_GPU2D_Init(void)
{

  /* USER CODE BEGIN GPU2D_Init 0 */

  /* USER CODE END GPU2D_Init 0 */

  /* USER CODE BEGIN GPU2D_Init 1 */

  /* USER CODE END GPU2D_Init 1 */
  hgpu2d.Instance = GPU2D;
  if (HAL_GPU2D_Init(&hgpu2d) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN GPU2D_Init 2 */

  /* USER CODE END GPU2D_Init 2 */

}

/**
  * @brief DSIHOST Initialization Function
  * @param None
  * @retval None
  */
void MX_DSIHOST_DSI_Init(void)
{

  /* USER CODE BEGIN DSIHOST_Init 0 */

  /* USER CODE END DSIHOST_Init 0 */

  DSI_PLLInitTypeDef PLLInit = {0};
  DSI_HOST_TimeoutTypeDef HostTimeouts = {0};
  DSI_PHY_TimerTypeDef PhyTimings = {0};
  DSI_VidCfgTypeDef VidCfg = {0};

  /* USER CODE BEGIN DSIHOST_Init 1 */

  /* USER CODE END DSIHOST_Init 1 */
  hdsi.Instance = DSI;
  hdsi.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  hdsi.Init.TXEscapeCkdiv = 4;
  hdsi.Init.NumberOfLanes = DSI_TWO_DATA_LANES;
  hdsi.Init.PHYFrequencyRange = DSI_DPHY_FRANGE_450MHZ_510MHZ;
  hdsi.Init.PHYLowPowerOffset = PHY_LP_OFFSSET_0_CLKP;
  PLLInit.PLLNDIV = 125;
  PLLInit.PLLIDF = DSI_PLL_IN_DIV4;
  PLLInit.PLLODF = DSI_PLL_OUT_DIV2;
  PLLInit.PLLVCORange = DSI_DPHY_VCO_FRANGE_800MHZ_1GHZ;
  PLLInit.PLLChargePump = DSI_PLL_CHARGE_PUMP_2000HZ_4400HZ;
  PLLInit.PLLTuning = DSI_PLL_LOOP_FILTER_2000HZ_4400HZ;
  if (HAL_DSI_Init(&hdsi, &PLLInit) != HAL_OK)
  {
    Error_Handler();
  }
  HostTimeouts.TimeoutCkdiv = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout = 0;
  HostTimeouts.HighSpeedReadTimeout = 0;
  HostTimeouts.LowPowerReadTimeout = 0;
  HostTimeouts.HighSpeedWriteTimeout = 0;
  HostTimeouts.HighSpeedWritePrespMode = DSI_HS_PM_DISABLE;
  HostTimeouts.LowPowerWriteTimeout = 0;
  HostTimeouts.BTATimeout = 0;
  if (HAL_DSI_ConfigHostTimeouts(&hdsi, &HostTimeouts) != HAL_OK)
  {
    Error_Handler();
  }
  PhyTimings.ClockLaneHS2LPTime = 11;
  PhyTimings.ClockLaneLP2HSTime = 40;
  PhyTimings.DataLaneHS2LPTime = 12;
  PhyTimings.DataLaneLP2HSTime = 23;
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime = 7;
  if (HAL_DSI_ConfigPhyTimer(&hdsi, &PhyTimings) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigFlowControl(&hdsi, DSI_FLOW_CONTROL_BTA) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&hdsi, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  VidCfg.ColorCoding = DSI_RGB888;
  VidCfg.LooselyPacked = DSI_LOOSELY_PACKED_DISABLE;
  VidCfg.Mode = DSI_VID_MODE_BURST;
  VidCfg.PacketSize = 480;
  VidCfg.NumberOfChunks = 0;
  VidCfg.NullPacketSize = 0;
  VidCfg.HSPolarity = DSI_HSYNC_ACTIVE_HIGH;
  VidCfg.VSPolarity = DSI_VSYNC_ACTIVE_HIGH;
  VidCfg.DEPolarity = DSI_DATA_ENABLE_ACTIVE_HIGH;
  VidCfg.HorizontalSyncActive = 6;
  VidCfg.HorizontalBackPorch = 3;
  VidCfg.HorizontalLine = 1452;
  VidCfg.VerticalSyncActive = 1;
  VidCfg.VerticalBackPorch = 12;
  VidCfg.VerticalFrontPorch = 50;
  VidCfg.VerticalActive = 481;
  VidCfg.LPCommandEnable = DSI_LP_COMMAND_DISABLE;
  VidCfg.LPLargestPacketSize = 0;
  VidCfg.LPVACTLargestPacketSize = 0;
  VidCfg.LPHorizontalFrontPorchEnable = DSI_LP_HFP_ENABLE;
  VidCfg.LPHorizontalBackPorchEnable = DSI_LP_HBP_ENABLE;
  VidCfg.LPVerticalActiveEnable = DSI_LP_VACT_ENABLE;
  VidCfg.LPVerticalFrontPorchEnable = DSI_LP_VFP_ENABLE;
  VidCfg.LPVerticalBackPorchEnable = DSI_LP_VBP_ENABLE;
  VidCfg.LPVerticalSyncActiveEnable = DSI_LP_VSYNC_ENABLE;
  VidCfg.FrameBTAAcknowledgeEnable = DSI_FBTAA_ENABLE;
  if (HAL_DSI_ConfigVideoMode(&hdsi, &VidCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_SetGenericVCID(&hdsi, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DSIHOST_Init 2 */
  LCD_Set_Default_Clock();
  /* USER CODE END DSIHOST_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  LTDC_LayerCfgTypeDef pLayerCfg = {0};

  /* USER CODE BEGIN LTDC_Init 1 */

  /* USER CODE END LTDC_Init 1 */
  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AH;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AH;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 1;
  hltdc.Init.VerticalSync = 0;
  hltdc.Init.AccumulatedHBP = 2;
  hltdc.Init.AccumulatedVBP = 12;
  hltdc.Init.AccumulatedActiveW = 482;
  hltdc.Init.AccumulatedActiveH = 493;
  hltdc.Init.TotalWidth = 483;
  hltdc.Init.TotalHeigh = 543;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = LCD_WIDTH;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = LCD_HEIGHT;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_ARGB8888;
  pLayerCfg.Alpha = 0xFF;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = LCD_FRAME_BUFFER;
  pLayerCfg.ImageWidth = LCD_WIDTH;
  pLayerCfg.ImageHeight = LCD_HEIGHT;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
void MX_LCD_Init(void)
{
#if defined(CF0F_PINMUX_ENABLED) && (CF0F_PINMUX_ENABLED == 1)
  if(SetPanelConfig_ST7701S() != 0)
#else
  if(SetPanelConfig() != 0)
#endif
  {
    Error_Handler();
  }

  /* Prepare area to display on LCD */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 50);

  /* Set UTIL_LCD functions */
  LCD_Driver.DrawBitmap  = 0;
  LCD_Driver.FillRGBRect = LCD_FillRGBRect;
  LCD_Driver.DrawHLine   = 0;
  LCD_Driver.DrawVLine   = 0;
  LCD_Driver.FillRect    = LCD_FillRect;
  LCD_Driver.GetPixel    = 0;
  LCD_Driver.SetPixel    = 0;
  LCD_Driver.GetXSize    = LCD_GetXSize;
  LCD_Driver.GetYSize    = LCD_GetYSize;
  LCD_Driver.SetLayer    = 0;
  LCD_Driver.GetFormat   = LCD_GetFormat;

  UTIL_LCD_SetFuncDriver(&LCD_Driver);

  /* Clear LCD */
  UTIL_LCD_Clear(UTIL_LCD_COLOR_BLACK);

  UTIL_LCD_SetBackColor(UTIL_LCD_COLOR_BLUE);
  UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
  UTIL_LCD_SetFont(&Font20);

  /* Display title */
  UTIL_LCD_FillRect(0, 70, LCD_WIDTH, 40, UTIL_LCD_COLOR_BLUE);
  UTIL_LCD_DisplayStringAt(0, 80, (uint8_t *) "Google CF0F Example", CENTER_MODE);
  //UTIL_LCD_DisplayStringAt(0, 100, (uint8_t *) " example ", CENTER_MODE);
  CopyBuffer((uint32_t *)Images[ImageIndex++], (uint32_t *)LCD_FRAME_BUFFER, 67, 140, IMAGE_WIDTH, IMAGE_HEIGHT);
}

/**
  * @brief JPEG Initialization Function
  * @param None
  * @retval None
  */
void MX_JPEG_Init(void)
{

  /* USER CODE BEGIN JPEG_Init 0 */

  /* USER CODE END JPEG_Init 0 */

  /* USER CODE BEGIN JPEG_Init 1 */

  /* USER CODE END JPEG_Init 1 */
  hjpeg.Instance = JPEG;
  if (HAL_JPEG_Init(&hjpeg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN JPEG_Init 2 */

  /* USER CODE END JPEG_Init 2 */

}

/* USER CODE END 0 */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */
/**
  * @brief DMA2D MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hdma2d: DMA2D handle pointer
  * @retval None
  */
void HAL_DMA2D_MspInit(DMA2D_HandleTypeDef* hdma2d)
{
  if(hdma2d->Instance==DMA2D)
  {
    /* USER CODE BEGIN DMA2D_MspInit 0 */

    /* USER CODE END DMA2D_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DMA2D_CLK_ENABLE();
    /* DMA2D interrupt Init */
    HAL_NVIC_SetPriority(DMA2D_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(DMA2D_IRQn);
    /* USER CODE BEGIN DMA2D_MspInit 1 */

    /* USER CODE END DMA2D_MspInit 1 */

  }

}

/**
  * @brief DMA2D MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hdma2d: DMA2D handle pointer
  * @retval None
  */
void HAL_DMA2D_MspDeInit(DMA2D_HandleTypeDef* hdma2d)
{
  if(hdma2d->Instance==DMA2D)
  {
    /* USER CODE BEGIN DMA2D_MspDeInit 0 */

    /* USER CODE END DMA2D_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DMA2D_CLK_DISABLE();

    /* DMA2D interrupt DeInit */
    HAL_NVIC_DisableIRQ(DMA2D_IRQn);
    /* USER CODE BEGIN DMA2D_MspDeInit 1 */

    /* USER CODE END DMA2D_MspDeInit 1 */
  }

}

/**
  * @brief DSI MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hdsi: DSI handle pointer
  * @retval None
  */
void HAL_DSI_MspInit(DSI_HandleTypeDef* hdsi)
{
  if(hdsi->Instance==DSI)
  {
    /* USER CODE BEGIN DSI_MspInit 0 */

    /* USER CODE END DSI_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_DSI_CLK_ENABLE();
    /* USER CODE BEGIN DSI_MspInit 1 */

    /* USER CODE END DSI_MspInit 1 */

  }

}

/**
  * @brief DSI MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hdsi: DSI handle pointer
  * @retval None
  */
void HAL_DSI_MspDeInit(DSI_HandleTypeDef* hdsi)
{
  if(hdsi->Instance==DSI)
  {
    /* USER CODE BEGIN DSI_MspDeInit 0 */

    /* USER CODE END DSI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_DSI_CLK_DISABLE();
    /* USER CODE BEGIN DSI_MspDeInit 1 */

    /* USER CODE END DSI_MspDeInit 1 */
  }

}

/**
  * @brief GPU2D MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hgpu2d: GPU2D handle pointer
  * @retval None
  */
void HAL_GPU2D_MspInit(GPU2D_HandleTypeDef* hgpu2d)
{
  if(hgpu2d->Instance==GPU2D)
  {
    /* USER CODE BEGIN GPU2D_MspInit 0 */

    /* USER CODE END GPU2D_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_GPU2D_CLK_ENABLE();
    __HAL_RCC_DCACHE2_CLK_ENABLE();
    /* GPU2D interrupt Init */
    HAL_NVIC_SetPriority(GPU2D_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(GPU2D_IRQn);
    HAL_NVIC_SetPriority(GPU2D_ER_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(GPU2D_ER_IRQn);
    /* USER CODE BEGIN GPU2D_MspInit 1 */

    /* USER CODE END GPU2D_MspInit 1 */

  }

}

/**
  * @brief GPU2D MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hgpu2d: GPU2D handle pointer
  * @retval None
  */
void HAL_GPU2D_MspDeInit(GPU2D_HandleTypeDef* hgpu2d)
{
  if(hgpu2d->Instance==GPU2D)
  {
    /* USER CODE BEGIN GPU2D_MspDeInit 0 */

    /* USER CODE END GPU2D_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_GPU2D_CLK_DISABLE();
    __HAL_RCC_DCACHE2_CLK_DISABLE();

    /* GPU2D interrupt DeInit */
    HAL_NVIC_DisableIRQ(GPU2D_IRQn);
    HAL_NVIC_DisableIRQ(GPU2D_ER_IRQn);
    /* USER CODE BEGIN GPU2D_MspDeInit 1 */

    /* USER CODE END GPU2D_MspDeInit 1 */
  }

}

/**
  * @brief JPEG MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hjpeg: JPEG handle pointer
  * @retval None
  */
void HAL_JPEG_MspInit(JPEG_HandleTypeDef* hjpeg)
{
  DMA_DataHandlingConfTypeDef DataHandlingConfig;
  if(hjpeg->Instance==JPEG)
  {
    /* USER CODE BEGIN JPEG_MspInit 0 */

    /* USER CODE END JPEG_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_JPEG_CLK_ENABLE();

    /* JPEG DMA Init */
    /* GPDMA1_REQUEST_JPEG_TX Init */
    handle_GPDMA1_Channel1.Instance = GPDMA1_Channel1;
    handle_GPDMA1_Channel1.Init.Request = GPDMA1_REQUEST_JPEG_TX;
    handle_GPDMA1_Channel1.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    handle_GPDMA1_Channel1.Init.SrcInc = DMA_SINC_FIXED;
    handle_GPDMA1_Channel1.Init.DestInc = DMA_DINC_INCREMENTED;
    handle_GPDMA1_Channel1.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_WORD;
    handle_GPDMA1_Channel1.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
    handle_GPDMA1_Channel1.Init.Priority = DMA_HIGH_PRIORITY;
    handle_GPDMA1_Channel1.Init.SrcBurstLength = 8;
    handle_GPDMA1_Channel1.Init.DestBurstLength = 8;
    handle_GPDMA1_Channel1.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT0|DMA_DEST_ALLOCATED_PORT1;
    handle_GPDMA1_Channel1.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel1.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hjpeg, hdmaout, handle_GPDMA1_Channel1);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel1, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* GPDMA1_REQUEST_JPEG_RX Init */
    handle_GPDMA1_Channel0.Instance = GPDMA1_Channel0;
    handle_GPDMA1_Channel0.Init.Request = GPDMA1_REQUEST_JPEG_RX;
    handle_GPDMA1_Channel0.Init.BlkHWRequest = DMA_BREQ_SINGLE_BURST;
    handle_GPDMA1_Channel0.Init.Direction = DMA_MEMORY_TO_PERIPH;
    handle_GPDMA1_Channel0.Init.SrcInc = DMA_SINC_INCREMENTED;
    handle_GPDMA1_Channel0.Init.DestInc = DMA_DINC_FIXED;
    handle_GPDMA1_Channel0.Init.SrcDataWidth = DMA_SRC_DATAWIDTH_BYTE;
    handle_GPDMA1_Channel0.Init.DestDataWidth = DMA_DEST_DATAWIDTH_WORD;
    handle_GPDMA1_Channel0.Init.Priority = DMA_HIGH_PRIORITY;
    handle_GPDMA1_Channel0.Init.SrcBurstLength = 8;
    handle_GPDMA1_Channel0.Init.DestBurstLength = 8;
    handle_GPDMA1_Channel0.Init.TransferAllocatedPort = DMA_SRC_ALLOCATED_PORT1|DMA_DEST_ALLOCATED_PORT0;
    handle_GPDMA1_Channel0.Init.TransferEventMode = DMA_TCEM_BLOCK_TRANSFER;
    handle_GPDMA1_Channel0.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&handle_GPDMA1_Channel0) != HAL_OK)
    {
      Error_Handler();
    }

    DataHandlingConfig.DataExchange = DMA_EXCHANGE_NONE;
    DataHandlingConfig.DataAlignment = DMA_DATA_PACK;
    if (HAL_DMAEx_ConfigDataHandling(&handle_GPDMA1_Channel0, &DataHandlingConfig) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hjpeg, hdmain, handle_GPDMA1_Channel0);

    if (HAL_DMA_ConfigChannelAttributes(&handle_GPDMA1_Channel0, DMA_CHANNEL_NPRIV) != HAL_OK)
    {
      Error_Handler();
    }

    /* JPEG interrupt Init */
    HAL_NVIC_SetPriority(JPEG_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(JPEG_IRQn);
    /* USER CODE BEGIN JPEG_MspInit 1 */

    /* USER CODE END JPEG_MspInit 1 */

  }

}

/**
  * @brief JPEG MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hjpeg: JPEG handle pointer
  * @retval None
  */
void HAL_JPEG_MspDeInit(JPEG_HandleTypeDef* hjpeg)
{
  if(hjpeg->Instance==JPEG)
  {
    /* USER CODE BEGIN JPEG_MspDeInit 0 */

    /* USER CODE END JPEG_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_JPEG_CLK_DISABLE();

    /* JPEG DMA DeInit */
    HAL_DMA_DeInit(hjpeg->hdmaout);
    HAL_DMA_DeInit(hjpeg->hdmain);

    /* JPEG interrupt DeInit */
    HAL_NVIC_DisableIRQ(JPEG_IRQn);
    /* USER CODE BEGIN JPEG_MspDeInit 1 */

    /* USER CODE END JPEG_MspDeInit 1 */
  }

}

/**
  * @brief LTDC MSP Initialization
  * This function configures the hardware resources used in this example
  * @param hltdc: LTDC handle pointer
  * @retval None
  */
void HAL_LTDC_MspInit(LTDC_HandleTypeDef* hltdc)
{
  if(hltdc->Instance==LTDC)
  {
    /* USER CODE BEGIN LTDC_MspInit 0 */

    /* USER CODE END LTDC_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_LTDC_CLK_ENABLE();
    /* LTDC interrupt Init */
    HAL_NVIC_SetPriority(LTDC_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(LTDC_IRQn);
    HAL_NVIC_SetPriority(LTDC_ER_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(LTDC_ER_IRQn);
    /* USER CODE BEGIN LTDC_MspInit 1 */

    /* USER CODE END LTDC_MspInit 1 */

  }

}

/**
  * @brief LTDC MSP De-Initialization
  * This function freeze the hardware resources used in this example
  * @param hltdc: LTDC handle pointer
  * @retval None
  */
void HAL_LTDC_MspDeInit(LTDC_HandleTypeDef* hltdc)
{
  if(hltdc->Instance==LTDC)
  {
    /* USER CODE BEGIN LTDC_MspDeInit 0 */

    /* USER CODE END LTDC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_LTDC_CLK_DISABLE();

    /* LTDC interrupt DeInit */
    HAL_NVIC_DisableIRQ(LTDC_IRQn);
    HAL_NVIC_DisableIRQ(LTDC_ER_IRQn);
    /* USER CODE BEGIN LTDC_MspDeInit 1 */

    /* USER CODE END LTDC_MspDeInit 1 */
  }

}


/******************************************************************************/
/*   USER IRQ HANDLER TREATMENT                                               */
/******************************************************************************/
