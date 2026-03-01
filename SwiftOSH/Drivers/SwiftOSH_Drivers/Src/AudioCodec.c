/**
  ******************************************************************************
  * @file    AudioCodec.c
  * @brief   TLV320ADC3120 codec driver — SwiftOSH (STM32U545RET6Q)
  *
  * The TLV320ADC3120 is NOT the same as the TLV320AIC3120.
  * It has a completely different register map and initialization sequence.
  *
  * Key facts from datasheet (SBASA91A):
  *  - I2C address: 0x4E (7-bit, ADDR pin high on SwiftOSH board)
  *  - Auto-detects BCLK/FSYNC and configures PLL internally — NO PLL registers
  *  - Default ASI format: TDM (P0_R7 = 0x30). Must set to I2S for SAI I2S mode.
  *
  * Gain register: CH1_CFG1 (P0_R61, address 0x3D)
  *  - Bits[7:1] = CH1_GAIN[6:0]: 0=0dB, 1=0.5dB, ..., 84=42dB (max). 85-127 reserved.
  *  - Bit[0] = CH1_GAIN_SIGN_BIT: 0=positive (normal)
  *  - Host sends gain as 0.5dB steps (0–95). ADC3120 max is 84 (42dB).
  *    Values above 84 are clamped. Register value is shifted left by 1 into bits[7:1].
  *
  * Digital volume: CH1_CFG2 (P0_R62, address 0x3E) — NOT the gain register.
  *  - Default 0xC9 = 0dB output volume. Leave at default.
  *
  * MICBIAS: BIAS_CFG (P0_R59, address 0x3B)
  *  - Bits[6:4] = MBIAS_VAL[2:0], Bits[1:0] = ADC_FSCALE[1:0]
  *  - Host sends: 0x00=off, 0x10=2.5V, 0x18=3.3V
  *  - 2.5V  → MBIAS_VAL=000, ADC_FSCALE=01 → BIAS_CFG=0x01
  *  - 3.3V  → MBIAS_VAL=110 (AVDD),          → BIAS_CFG=0x60
  *  - off   → BIAS_CFG=0x00, PWR_CFG omits MICBIAS_PDZ bit
  *
  * Initialization sequence (section 9.2.1.2):
  *  1. Write P0_R2  = 0x81  — wake from sleep, use internal AREG (AVDD=3.3V)
  *  2. Wait >= 1 ms
  *  3. Write P0_R7  = 0x40  — I2S mode, 16-bit word length
  *  4. Write P0_R59 = bias  — MICBIAS voltage (or 0x00 if off)
  *  5. Write P0_R60 = 0x20  — CH1 single-ended input (IN1P/IN1M)
  *  6. Write P0_R61 = gain  — CH1 analog gain (value << 1 into bits[7:1])
  *  7. Write P0_R115 = 0x80 — enable input channel 1 only
  *  8. Write P0_R116 = 0x80 — enable ASI output channel 1 only
  *  9. Apply FSYNC and BCLK (SAI master starts clocking)
  * 10. Write P0_R117 = PWR  — power up ADC + PLL (+ MICBIAS if enabled)
  *
  * SAI configuration (main.c):
  *  - Master RX, I2S, sample rate from flash, 2 slots, 32-bit slot
  *  - BCLK = 64 × SampleRate (Table 8-6: ratio=64, all rates supported)
  ******************************************************************************
  */
#include "stm32u5xx_hal.h"
#include "AudioCodec.h"
#include "GeneralDefines.h"

/* I2C address: 7-bit 0x4E shifted left for HAL (8-bit write address = 0x9C) */
#define ADC3120_ADDR   (CODEC_I2C_ADDRESS)   /* 0x4E << 1 = 0x9C */

/* Page 0 register addresses */
#define REG_PAGE_CFG        0x00   /* Page select */
#define REG_SW_RESET        0x01   /* Software reset (self-clearing) */
#define REG_SLEEP_CFG       0x02   /* Sleep mode + AREG select */
#define REG_ASI_CFG0        0x07   /* ASI format + word length */
#define REG_BIAS_CFG        0x3B   /* BIAS_CFG: MBIAS_VAL[6:4], ADC_FSCALE[1:0] (P0_R59) */
#define REG_CH1_CFG0        0x3C   /* Channel 1 config 0: input source, impedance (P0_R60) */
#define REG_CH1_CFG1        0x3D   /* Channel 1 config 1: analog gain bits[7:1] (P0_R61) */
#define REG_IN_CH_EN        0x73   /* Input channel enable (P0_R115) */
#define REG_ASI_OUT_CH_EN   0x74   /* ASI output channel enable (P0_R116) */
#define REG_PWR_CFG         0x75   /* Power-up: ADC, MICBIAS, PLL (P0_R117) */

/* SLEEP_CFG values */
#define SLEEP_CFG_ACTIVE_INTERNAL_AREG  0x81  /* AREG_SELECT=1 (internal, for AVDD=3.3V), SLEEP_ENZ=1 (active) */
#define SLEEP_CFG_SLEEP                 0x00  /* SLEEP_ENZ=0 → sleep mode */

/* ASI_CFG0: I2S mode (bits[7:6]=01), 16-bit word (bits[5:4]=00) → 0x40
   Default is TDM (0x30). SAI is configured as I2S master so codec must be I2S slave. */
#define ASI_CFG0_I2S_16BIT  0x40

/* CH1_CFG0 (P0_R60): single-ended input, IN1P/IN1M
   CH1_INSRC[1:0] = bits[6:5] = 01 → single-ended */
#define CH1_CFG0_SINGLE_ENDED  0x20

/* Channel enables — mono: channel 1 only */
#define IN_CH_EN_CH1_ONLY   0x80   /* IN_CH1_EN=1, IN_CH2_EN=0 */
#define ASI_OUT_CH1_ONLY    0x80   /* ASI_OUT_CH1_EN=1, ASI_OUT_CH2_EN=0 */

/* PWR_CFG bits */
#define PWR_CFG_ADC_PLL          0xA0  /* ADC_PDZ=1, PLL_PDZ=1 (no MICBIAS) */
#define PWR_CFG_ADC_MICBIAS_PLL  0xE0  /* ADC_PDZ=1, MICBIAS_PDZ=1, PLL_PDZ=1 */

/* Write a single register on page 0 */
static HAL_StatusTypeDef Codec_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = { reg, val };
  return HAL_I2C_Master_Transmit(hi2c, ADC3120_ADDR, buf, 2, 100);
}

/**
  * @brief  Translate host mic bias byte to ADC3120 BIAS_CFG register value.
  *
  *  Host value  Meaning    BIAS_CFG
  *  0x00        off        0x00  (MBIAS_VAL=000, ADC_FSCALE=00 → 2.75V but MICBIAS_PDZ not set)
  *  0x10        2.5V       0x01  (MBIAS_VAL=000, ADC_FSCALE=01 → VREF×1 at 2.5V)
  *  0x18        3.3V       0x60  (MBIAS_VAL=110 → AVDD ≈ 3.3V)
  */
static uint8_t TranslateMicBias(uint8_t host_val)
{
  switch (host_val)
  {
    case 0x10: return 0x01;  /* 2.5V: MBIAS_VAL=000, ADC_FSCALE=01 */
    case 0x18: return 0x60;  /* 3.3V: MBIAS_VAL=110 (AVDD) */
    default:   return 0x00;  /* off or unknown: leave at default */
  }
}

/**
  * @brief  Initialize TLV320ADC3120 for recording.
  *         Call AudioCodec_PowerUp() AFTER HAL_SAI DMA starts clocks.
  * @param  hi2c         I2C handle
  * @param  gain         Host gain byte: 0.5dB/step, 0–95. Clamped to 84 (42dB max).
  * @param  mic_bias_cfg Host mic bias byte: 0x00=off, 0x10=2.5V, 0x18=3.3V
  */
unsigned char AudioCodec_Initialize(I2C_HandleTypeDef *hi2c, uint8_t gain, uint8_t mic_bias_cfg)
{
  /* 1. Software reset — restores all registers to POR defaults */
  Codec_WriteReg(hi2c, REG_SW_RESET, 0x01);
  HAL_Delay(2);

  /* 2. Wake from sleep, enable internal AREG regulator (required for AVDD=3.3V) */
  Codec_WriteReg(hi2c, REG_SLEEP_CFG, SLEEP_CFG_ACTIVE_INTERNAL_AREG);
  HAL_Delay(2);  /* >= 1 ms required after wake-up */

  /* 3. Set ASI format to I2S, 16-bit word length */
  Codec_WriteReg(hi2c, REG_ASI_CFG0, ASI_CFG0_I2S_16BIT);

  /* 4. Set MICBIAS voltage (or leave at default if off) */
  Codec_WriteReg(hi2c, REG_BIAS_CFG, TranslateMicBias(mic_bias_cfg));

  /* 5. Configure channel 1 as single-ended input (IN1P/IN1M) */
  Codec_WriteReg(hi2c, REG_CH1_CFG0, CH1_CFG0_SINGLE_ENDED);

  /* 6. Set analog gain in CH1_CFG1 (P0_R61), bits[7:1].
     Host sends 0.5dB steps (0–95). ADC3120 max is 84 (42dB); 85–127 reserved.
     Register format: gain value sits in bits[7:1], bit[0]=sign (0=positive). */
  if (gain > 84) gain = 84;
  Codec_WriteReg(hi2c, REG_CH1_CFG1, (uint8_t)(gain << 1));

  /* 7. Enable input channel 1 only */
  Codec_WriteReg(hi2c, REG_IN_CH_EN, IN_CH_EN_CH1_ONLY);

  /* 8. Enable ASI output slot for channel 1 only */
  Codec_WriteReg(hi2c, REG_ASI_OUT_CH_EN, ASI_OUT_CH1_ONLY);

  /* NOTE: PWR_CFG is NOT written here — call AudioCodec_PowerUp() after SAI DMA starts.
     The codec PLL auto-detects BCLK/FSYNC; clocks must be present before power-up
     or the PLL will fail to lock. */
  return 0;
}

/**
  * @brief  Power up ADC, PLL, and optionally MICBIAS — call AFTER SAI DMA is started.
  * @param  hi2c         I2C handle
  * @param  mic_bias_cfg Host mic bias byte: 0x00=off → no MICBIAS_PDZ; else include it
  */
unsigned char AudioCodec_PowerUp(I2C_HandleTypeDef *hi2c, uint8_t mic_bias_cfg)
{
  uint8_t pwr = (mic_bias_cfg != 0x00) ? PWR_CFG_ADC_MICBIAS_PLL : PWR_CFG_ADC_PLL;
  Codec_WriteReg(hi2c, REG_PWR_CFG, pwr);
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
  * @param  mic_bias_cfg Host mic bias byte (same value used at init)
  */
unsigned char AudioCodec_WakeFromSleep(I2C_HandleTypeDef *hi2c, uint8_t mic_bias_cfg)
{
  /* Wake from sleep — internal AREG, active mode */
  Codec_WriteReg(hi2c, REG_SLEEP_CFG, SLEEP_CFG_ACTIVE_INTERNAL_AREG);
  HAL_Delay(2);  /* >= 1 ms wake-up sequence */

  /* Re-enable channel 1 and power — registers are retained through sleep */
  Codec_WriteReg(hi2c, REG_IN_CH_EN, IN_CH_EN_CH1_ONLY);
  Codec_WriteReg(hi2c, REG_ASI_OUT_CH_EN, ASI_OUT_CH1_ONLY);

  uint8_t pwr = (mic_bias_cfg != 0x00) ? PWR_CFG_ADC_MICBIAS_PLL : PWR_CFG_ADC_PLL;
  Codec_WriteReg(hi2c, REG_PWR_CFG, pwr);

  return 0;
}
