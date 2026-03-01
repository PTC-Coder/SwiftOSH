/**
  ******************************************************************************
  * @file    AudioCodec.c
  * @brief   TLV320ADC3120 codec driver — SwiftOSH (STM32U545RET6Q)
  *
  * The TLV320ADC3120 is NOT the same as the TLV320AIC3120.
  * It has a completely different register map and initialization sequence.
  *
  * Key facts from datasheet (SBASA91A):
  *  - I2C address: 0x4D (7-bit), fixed — cannot be changed
  *  - Auto-detects BCLK/FSYNC and configures PLL internally — NO PLL registers
  *  - Default ASI format: TDM (P0_R7 = 0x30). Must set to I2S for SAI I2S mode.
  *  - Initialization sequence (section 9.2.1.2):
  *      1. Write P0_R2  = 0x81  — wake from sleep, use internal AREG (AVDD=3.3V)
  *      2. Wait >= 1 ms
  *      3. Write P0_R7  = 0x70  — I2S mode, 16-bit word length
  *      4. Write P0_R115 = 0xC0 — enable input channels 1 and 2
  *      5. Write P0_R116 = 0xC0 — enable ASI output channels 1 and 2
  *      6. Write P0_R117 = 0xE0 — power up ADC + MICBIAS + PLL
  *      7. Apply FSYNC and BCLK (SAI master starts clocking)
  *
  * SAI configuration (main.c):
  *  - Master RX, I2S-like, 32kHz, 2 slots, 32-bit slot, SlotActive=0
  *  - BCLK = 64 × 32000 = 2.048 MHz  (Table 8-6: FSYNC=32kHz, ratio=64 → supported)
  *
  * Sleep mode: write P0_R2 = 0x00 (SLEEP_ENZ=0, AREG_SELECT=0)
  * Wake from sleep: write P0_R2 = 0x81 again, wait 1ms
  ******************************************************************************
  */
#include "stm32u5xx_hal.h"
#include "AudioCodec.h"
#include "GeneralDefines.h"

/* I2C address: 7-bit 0x4D shifted left for HAL (8-bit write address = 0x9A) */
#define ADC3120_ADDR   (CODEC_I2C_ADDRESS)   /* 0x4D << 1 = 0x9A */

/* Page 0 register addresses */
#define REG_PAGE_CFG        0x00   /* Page select */
#define REG_SW_RESET        0x01   /* Software reset (self-clearing) */
#define REG_SLEEP_CFG       0x02   /* Sleep mode + AREG select */
#define REG_ASI_CFG0        0x07   /* ASI format + word length */
#define REG_BIAS_CFG        0x3B   /* BIAS_CFG: MBIAS_VAL[6:4], ADC_FSCALE[1:0] (P0_R59) */
#define REG_CH1_CFG0        0x3C   /* Channel 1 config 0: input source, impedance (P0_R60) */
#define REG_CH1_CFG2        0x3E   /* Channel 1 config 2: volume/gain (P0_R62) */
#define REG_IN_CH_EN        0x73   /* Input channel enable (P0_R115) */
#define REG_ASI_OUT_CH_EN   0x74   /* ASI output channel enable (P0_R116) */
#define REG_PWR_CFG         0x75   /* Power-up: ADC, MICBIAS, PLL (P0_R117) */

/* SLEEP_CFG values */
#define SLEEP_CFG_ACTIVE_INTERNAL_AREG  0x81  /* AREG_SELECT=1 (internal, for AVDD=3.3V), SLEEP_ENZ=1 (active) */
#define SLEEP_CFG_SLEEP                 0x00  /* SLEEP_ENZ=0 → sleep mode */

/* ASI_CFG0: I2S mode (bits[7:6]=01), 16-bit word (bits[5:4]=00) → 0x40
   Default is TDM (0x30). SAI is configured as I2S master so codec must be I2S slave.
   Word length bits[5:4]: 00=16-bit, 01=20-bit, 10=24-bit, 11=32-bit.
   SAI DataSize=SAI_DATASIZE_16 → codec must output 16-bit words to match. */
#define ASI_CFG0_I2S_16BIT  0x40

/* BIAS_CFG (P0_R59): MICBIAS voltage and ADC full-scale reference
   Per TLV320ADC3120 datasheet:
   - MBIAS_VAL[6:4]: 000=VREF, 001=VREFx1.096, 2-7=reserved
   - ADC_FSCALE[1:0]: 00=2.75V, 01=2.5V, 10=1.375V
   Default 0x00 = MICBIAS at VREF (2.75V), ADC ref 2.75V */
#define BIAS_CFG_DEFAULT  0x00

/* CH1_CFG0 (P0_R60): single-ended input, IN1P signal / IN1M grounded
   CH1_INSRC[1:0] = bits[6:5] = 01 → single-ended */
#define CH1_CFG0_SINGLE_ENDED  0x20

/* Channel enables — mono: channel 1 only */
#define IN_CH_EN_CH1_ONLY   0x80   /* IN_CH1_EN=1, IN_CH2_EN=0 */
#define ASI_OUT_CH1_ONLY    0x80   /* ASI_OUT_CH1_EN=1, ASI_OUT_CH2_EN=0 */

/* PWR_CFG: MICBIAS_PDZ=1, ADC_PDZ=1, PLL_PDZ=1 → 0xE0 */
#define PWR_CFG_ADC_MICBIAS_PLL  0xE0

/* Write a single register on page 0 */
static HAL_StatusTypeDef Codec_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = { reg, val };
  return HAL_I2C_Master_Transmit(hi2c, ADC3120_ADDR, buf, 2, 100);
}

/**
  * @brief  Initialize TLV320ADC3120 for recording.
  *         Call AudioCodec_PowerUp() AFTER HAL_SAI_Receive_DMA() starts clocks.
  * @param  hi2c   I2C handle
  * @param  gain   Digital gain for CH1_CFG2: 0x00=0dB, each step=0.5dB (0x38=28dB)
  */
unsigned char AudioCodec_Initialize(I2C_HandleTypeDef *hi2c, uint8_t gain)
{
  /* 1. Software reset — restores all registers to POR defaults */
  Codec_WriteReg(hi2c, REG_SW_RESET, 0x01);
  HAL_Delay(2);

  /* 2. Wake from sleep, enable internal AREG regulator (required for AVDD=3.3V) */
  Codec_WriteReg(hi2c, REG_SLEEP_CFG, SLEEP_CFG_ACTIVE_INTERNAL_AREG);
  HAL_Delay(2);  /* >= 1 ms required after wake-up */

  /* 3. Set ASI format to I2S, 16-bit word length */
  Codec_WriteReg(hi2c, REG_ASI_CFG0, ASI_CFG0_I2S_16BIT);

  /* 4. Configure channel 1 as single-ended input (IN1P/IN1M) */
  Codec_WriteReg(hi2c, REG_CH1_CFG0, CH1_CFG0_SINGLE_ENDED);

  /* 5. Set MICBIAS and ADC full-scale reference to defaults (2.75V) */
  Codec_WriteReg(hi2c, REG_BIAS_CFG, BIAS_CFG_DEFAULT);

  /* 6. Set digital gain: 0x00=0dB, 0.5dB/step → 0x38=28dB, 0x28=20dB */
  Codec_WriteReg(hi2c, REG_CH1_CFG2, gain);

  /* 7. Enable input channel 1 only */
  Codec_WriteReg(hi2c, REG_IN_CH_EN, IN_CH_EN_CH1_ONLY);

  /* 8. Enable ASI output slot for channel 1 only */
  Codec_WriteReg(hi2c, REG_ASI_OUT_CH_EN, ASI_OUT_CH1_ONLY);

  /* NOTE: PWR_CFG is NOT written here — call AudioCodec_PowerUp() after SAI DMA starts */
  return 0;
}

/**
  * @brief  Power up ADC, MICBIAS, and PLL — call AFTER SAI DMA is started.
  *         The codec PLL auto-detects BCLK/FSYNC; clocks must be present
  *         before this is called or the PLL will fail to lock.
  */
unsigned char AudioCodec_PowerUp(I2C_HandleTypeDef *hi2c)
{
  Codec_WriteReg(hi2c, REG_PWR_CFG, PWR_CFG_ADC_MICBIAS_PLL);
  return 0;
}

/**
  * @brief  Put codec into sleep mode (low-power, < 10 µA).
  *         Wait >= 6 ms after calling before stopping BCLK/FSYNC.
  */
unsigned char AudioCodec_SleepMode(I2C_HandleTypeDef *hi2c)
{
  Codec_WriteReg(hi2c, REG_SLEEP_CFG, SLEEP_CFG_SLEEP);
  HAL_Delay(10);  /* Allow volume ramp-down and blocks to power down */
  return 0;
}

/**
  * @brief  Wake codec from sleep and resume recording.
  *         BCLK/FSYNC must already be running (SAI DMA active).
  */
unsigned char AudioCodec_WakeFromSleep(I2C_HandleTypeDef *hi2c)
{
  /* Wake from sleep — internal AREG, active mode */
  Codec_WriteReg(hi2c, REG_SLEEP_CFG, SLEEP_CFG_ACTIVE_INTERNAL_AREG);
  HAL_Delay(2);  /* >= 1 ms wake-up sequence */

  /* Re-enable channel 1 and power — registers are retained through sleep */
  Codec_WriteReg(hi2c, REG_IN_CH_EN, IN_CH_EN_CH1_ONLY);
  Codec_WriteReg(hi2c, REG_ASI_OUT_CH_EN, ASI_OUT_CH1_ONLY);
  Codec_WriteReg(hi2c, REG_PWR_CFG, PWR_CFG_ADC_MICBIAS_PLL);

  return 0;
}
