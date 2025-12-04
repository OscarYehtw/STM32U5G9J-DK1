/**
 * @file lm3697.c
 *
 * This file provides a basic structure to control the TI LM3697 external I2C backlight driver IC.
 */

#include "stm32u5xx_hal.h"
#include "lm3697.h"
#include <stdint.h>
#include <stdio.h>

// Assuming your I2C instance name is hbus_i2c4
extern I2C_HandleTypeDef hbus_i2c4;
int backlight_init = 0;

// ====================================================================
// Internal Driver Helper Functions
// ====================================================================

/**
 * @brief Writes a single register value to the LM3697 via I2C.
 *
 * @param RegAddress Register address to write (8-bit).
 * @param Value Data value to write (8-bit).
 * @return HAL_StatusTypeDef HAL status code.
 */
static HAL_StatusTypeDef LM3697_I2C_WriteReg(uint8_t RegAddress, uint8_t Value)
{
    // I2C write function buffer [Register Address, Data Value]
    uint8_t aData[2];
    aData[0] = RegAddress;
    aData[1] = Value;

    // Use HAL_I2C_Master_Transmit for transmission
    // Parameters: I2C Handle, Device Address, Data Buffer, Data Length, Timeout
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hbus_i2c4,
                                                     LM3697_I2C_ADDRESS_8BIT,
                                                     aData,
                                                     2,
                                                     100); // Set 100ms timeout

    if (status != HAL_OK) {
        printf("LM3697 I2C Write Failed (Status: 0x%02X, Reg: 0x%02X)\n", status, RegAddress);
    }
    return status;
}

// ===========================================
// Driver API Functions
// ===========================================

/**
 * @brief Set the lm3697 backlight brightness. Only modify brightness MSB.(8 bit mode)
 *
 * @return HAL_StatusTypeDef HAL status code.
 */
HAL_StatusTypeDef BL_Driver_SetBrightness_8bit(int brightness)
{
	HAL_StatusTypeDef status = HAL_OK;
	if (backlight_init == 0)
		BL_Driver_Init();
	// value : 0 <= value <= 255
	if (brightness > LM3697_MAX_BRIGHTNESS) brightness = LM3697_MAX_BRIGHTNESS;
	if (brightness < LM3697_MIN_BRIGHTNESS) brightness = LM3697_MIN_BRIGHTNESS;

	printf("brightness sets to %d\n", brightness);
	//LSB do not use in 8 bit mode.
	status = LM3697_I2C_WriteReg(LM3697_CTRL_A_Brightness_LSB, LM3697_MIN_BRIGHTNESS);
	status = LM3697_I2C_WriteReg(LM3697_CTRL_A_Brightness_MSB, brightness);
	return status;
}

/**
 * @brief Initializes the LM3697 backlight driver IC.
 * 		  Directly set to max brightness for test.
 *
 * @return HAL_StatusTypeDef HAL status code.
 */
HAL_StatusTypeDef BL_Driver_Init(void)
{
    HAL_StatusTypeDef status = HAL_OK;

    // Set backlight drive IC enable.
    // HAL_GPIO_WritePin(MCU_DISP_BL_EN_Port, MCU_DISP_BL_EN_Pin, GPIO_PIN_SET);

    printf("LM3697 initial starting...\n");
    // Use Control Bank A to set CH1 and CH2.
    if ((status = LM3697_I2C_WriteReg(LM3697_HVLED_Current_Sink_Output, LM3697_USE_Bank_A_CH1_CH2)) != HAL_OK)
        return status;
    // Enable CH1 and CH2 feedback. Disable CH3 feedback.
    if ((status = LM3697_I2C_WriteReg(LM3697_HVLED_Current_Sink_Feedback_EN, LM3697_EN_CH1_CH2_DIS_CH3)) != HAL_OK)
        return status;
    // Disable PWM.
    if ((status = LM3697_I2C_WriteReg(LM3697_PWM_Configuration, LM3697_DIS_PWM)) != HAL_OK)
        return status;
    // Initial max brightness LSB, use 11 bits mode.
    if ((status = LM3697_I2C_WriteReg(LM3697_CTRL_A_Brightness_LSB, LM3697_brightness_LSB_MAX)) != HAL_OK)
        return status;
    // Initial max brightness MSB, use 11 bits mode.
    if ((status = LM3697_I2C_WriteReg(LM3697_CTRL_A_Brightness_MSB, LM3697_brightness_MSB_MAX)) != HAL_OK)
        return status;
    // Enable backlight.
    if ((status = LM3697_I2C_WriteReg(LM3697_CTRL_Bank_EN, LM3697_backlight_EN)) != HAL_OK)
        return status;

    if (status == HAL_OK) {
        printf("LM3697 Backlight Driver initialization successful.\n");
        backlight_init = 1;
    }

    return status;
}
