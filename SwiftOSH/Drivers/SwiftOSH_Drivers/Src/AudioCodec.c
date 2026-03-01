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
#define REG_MICBIAS_CFG     0x3B   /* MICBIAS configuration (P0_R59) */
#define REG_CH1_CFG0        0x3C   /* Channel 1 config 0: input source, impedance (P0_R60) */
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

/* MICBIAS_CFG (P0_R59): Enable MICBIAS output with 2.75V
   MICBIAS_VOUT_SEL[2:0] = bits[2:0]: 000=AVDD, 001=VREF, 010-111 = various voltages
   MICBIAS_SRC_IMP[1:0] = bits[5:4]: source impedance
   MICBIAS_OUT_EN = bit 7: 1 = enable MICBIAS output
   For electret mic with 3.3V AVDD: use 2.75V (0b101) → 0x85 */
#define MICBIAS_CFG_ENABLE_2V75  0x85

/* CH1_CFG0 (P0_R60): single-ended input, IN1P signal / IN1M grounded
   CH1_INSRC[1:0] = bits[6:5] = 01 → single-ended
   All other bits default (0): 10kΩ impedance, AC-coupled, no DC offset */
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

/* Read a single register on page 0 */
static HAL_StatusTypeDef Codec_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *val)
{
  HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(hi2c, ADC3120_ADDR, &reg, 1, 100);
  if (st != HAL_OK) return st;
  return HAL_I2C_Master_Receive(hi2c, ADC3120_ADDR, val, 1, 100);
}

/**
  * @brief  Initialize TLV320ADC3120 for 2-channel analog recording.
  *         SAI must be configured as I2S master before calling this.
  *         BCLK and FSYNC will be applied by HAL_SAI_Receive_DMA().
  */
/* Global diagnostic: last read-back value for debug logging */
uint8_t g_codecReadback = 0xFF;
HAL_StatusTypeDef g_codecI2cStatus = HAL_OK;

unsigned char AudioCodec_Initialize(I2C_HandleTypeDef *hi2c)
{
  /* 1. Software reset — restores all registers to POR defaults */
  g_codecI2cStatus = Codec_WriteReg(hi2c, REG_SW_RESET, 0x01);
  HAL_Delay(2);

  /* 2. Wake from sleep, enable internal AREG regulator (required for AVDD=3.3V) */
  Codec_WriteReg(hi2c, REG_SLEEP_CFG, SLEEP_CFG_ACTIVE_INTERNAL_AREG);
  HAL_Delay(2);  /* >= 1 ms required after wake-up */

  /* 3. Set ASI format to I2S, 16-bit word length.
     Default (0x30) is TDM mode — must change to I2S to match SAI config.
     16-bit word matches SAI DataSize=SAI_DATASIZE_16. */
  Codec_WriteReg(hi2c, REG_ASI_CFG0, ASI_CFG0_I2S_16BIT);

  /* 3b. Configure channel 1 as single-ended input (IN1P signal, IN1M grounded).
     Default is differential — wrong for this schematic. */
  Codec_WriteReg(hi2c, REG_CH1_CFG0, CH1_CFG0_SINGLE_ENDED);

  /* 3c. Enable MICBIAS output at 2.75V for electret microphone.
     Without this, the microphone has no bias voltage and outputs no signal. */
  Codec_WriteReg(hi2c, REG_MICBIAS_CFG, MICBIAS_CFG_ENABLE_2V75);

  /* 4. Enable input channel 1 only */
  Codec_WriteReg(hi2c, REG_IN_CH_EN, IN_CH_EN_CH1_ONLY);

  /* Read back MICBIAS_CFG to verify I2C is working */
  Codec_ReadReg(hi2c, REG_MICBIAS_CFG, &g_codecReadback);

  /* 5. Enable ASI output slot for channel 1 only */
  Codec_WriteReg(hi2c, REG_ASI_OUT_CH_EN, ASI_OUT_CH1_ONLY);

  /* NOTE: PWR_CFG (step 6) is NOT written here.
     The codec PLL needs BCLK/FSYNC present when it powers up, otherwise it
     cannot lock and will not output data. Call AudioCodec_PowerUp() AFTER
     HAL_SAI_Receive_DMA() has started the SAI clocks. */

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
