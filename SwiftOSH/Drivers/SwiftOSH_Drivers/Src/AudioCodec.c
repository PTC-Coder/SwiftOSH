/**
  ******************************************************************************
  * @file    AudioCodec.c
  * @brief   TLV320ADC3120 codec driver — SwiftOSH (STM32U545RET6Q)
  *          Ported from SwiftOne AudioCodec.c.
  *          Configures codec via I2C for audio capture.
  ******************************************************************************
  */
#include "stm32u5xx_hal.h"
#include "AudioCodec.h"
#include "GeneralDefines.h"
#include "SwiftSettings.h"

/* I2C register write helper */
static HAL_StatusTypeDef Codec_WriteReg(I2C_HandleTypeDef *hi2c,
                                         uint8_t page, uint8_t reg, uint8_t val)
{
  uint8_t buf[2];

  /* Select page */
  buf[0] = 0x00;  /* Page select register */
  buf[1] = page;
  if (HAL_I2C_Master_Transmit(hi2c, CODEC_I2C_ADDRESS, buf, 2, 100) != HAL_OK)
    return HAL_ERROR;

  /* Write register */
  buf[0] = reg;
  buf[1] = val;
  return HAL_I2C_Master_Transmit(hi2c, CODEC_I2C_ADDRESS, buf, 2, 100);
}

/**
  * @brief  Initialize codec with settings from flash.
  */
unsigned char AudioCodec_Initialize(I2C_HandleTypeDef *hi2c)
{
  Codec_Config cfg;
  SwiftSettings_GetCodecVariables(&cfg);

  /* Software reset */
  Codec_WriteReg(hi2c, 0, 1, 0x01);
  HAL_Delay(10);

  /* PLL configuration (Page 0) */
  Codec_WriteReg(hi2c, 0, 4, cfg.Codec_PLL_PR);   /* PLL P & R */
  Codec_WriteReg(hi2c, 0, 5, cfg.Codec_PLL_J);     /* PLL J */
  Codec_WriteReg(hi2c, 0, 6, cfg.Codec_PLL_D_MSB); /* PLL D MSB */
  Codec_WriteReg(hi2c, 0, 7, cfg.Codec_PLL_D_LSB); /* PLL D LSB */

  /* ADC clock dividers */
  Codec_WriteReg(hi2c, 0, 18, cfg.Codec_NADC);  /* NADC */
  Codec_WriteReg(hi2c, 0, 19, cfg.Codec_MADC);  /* MADC */
  Codec_WriteReg(hi2c, 0, 20, cfg.Codec_AOSR);  /* AOSR */

  /* Audio interface: I2S, 16-bit, BCLK/WCLK inputs */
  Codec_WriteReg(hi2c, 0, 27, 0x00);

  /* Processing block */
  Codec_WriteReg(hi2c, 0, 61, cfg.Codec_PRB);

  /* Page 1: Mic bias and gain */
  Codec_WriteReg(hi2c, 1, 51, cfg.Codec_MIC_BIAS_CFG);  /* Mic bias */
  Codec_WriteReg(hi2c, 1, 52, cfg.Codec_GAIN);           /* Left ADC gain */
  Codec_WriteReg(hi2c, 1, 54, 0x00);                     /* Left ADC input */

  /* Power up ADC */
  Codec_WriteReg(hi2c, 0, 81, 0x80);  /* ADC power up */
  Codec_WriteReg(hi2c, 0, 82, 0x00);  /* ADC unmute */

  return 0;
}

/**
  * @brief  Put codec into low-power sleep mode.
  */
unsigned char AudioCodec_SleepMode(I2C_HandleTypeDef *hi2c)
{
  /* Power down ADC */
  Codec_WriteReg(hi2c, 0, 81, 0x00);
  return 0;
}

/**
  * @brief  Wake codec from sleep mode.
  */
unsigned char AudioCodec_WakeFromSleep(I2C_HandleTypeDef *hi2c)
{
  /* Power up ADC */
  Codec_WriteReg(hi2c, 0, 81, 0x80);
  Codec_WriteReg(hi2c, 0, 82, 0x00);
  return 0;
}
