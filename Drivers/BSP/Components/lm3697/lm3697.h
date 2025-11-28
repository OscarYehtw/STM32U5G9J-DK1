/**
  ******************************************************************************
  * @file    lm3697.h
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef LM3697_H
#define LM3697_H

// LM3697 7-bit I2C Slave Address
#define LM3697_I2C_ADDRESS_7BIT      0x3C

// Convert to 8-bit address (write mode)
#define LM3697_I2C_ADDRESS_8BIT      (LM3697_I2C_ADDRESS_7BIT << 1)

// LM3697 Register Definitions (Consult datasheet to confirm)
#define LM3697_HVLED_Current_Sink_Output              0x10
#define LM3697_HVLED_Current_Sink_Feedback_EN         0x19
#define LM3697_PWM_Configuration  			              0x1C
#define LM3697_CTRL_A_Brightness_LSB  			  	      0x20
#define LM3697_CTRL_A_Brightness_MSB  			  	      0x21
#define LM3697_CTRL_Bank_EN	  			      	          0x24

// LM3697 Register MAX brightness value
#define LM3697_USE_Bank_A_CH1_CH2                     0x04
#define LM3697_EN_CH1_CH2_DIS_CH3                     0x03
#define LM3697_DIS_PWM                                0x00
#define LM3697_brightness_LSB_MAX                     0x07
#define LM3697_brightness_MSB_MAX                     0xFF
#define LM3697_backlight_EN                           0x01

#define LM3697_MAX_BRIGHTNESS                         0xFF
#define LM3697_MIN_BRIGHTNESS                         0x00

#endif /* LM3697_H */
