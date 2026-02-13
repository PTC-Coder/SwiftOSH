/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c — SwiftOSH STM32U545RET6Q
  ******************************************************************************
  */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"
#include "error.h"

void Error_Handler(void);

/* ---- Pin definitions from Hardware_Desc.txt (STM32U545RET6Q) ---- */

/* LEDs */
#define RED_LED_Pin          GPIO_PIN_1
#define RED_LED_GPIO_Port    GPIOC
#define GRN_LED_Pin          GPIO_PIN_1
#define GRN_LED_GPIO_Port    GPIOA
#define BLU_LED_Pin          GPIO_PIN_2
#define BLU_LED_GPIO_Port    GPIOA

/* Pushbutton (PC6) */
#define PUSHBUTTON_Pin       GPIO_PIN_6
#define PUSHBUTTON_GPIO_Port GPIOC

/* Hall-effect switch (PC0) */
#define HALL_EFFECT_Pin          GPIO_PIN_0
#define HALL_EFFECT_GPIO_Port    GPIOC

/* SDMMC1 card-detect (PC13) */
#define SDCARD_DETECT_Pin        GPIO_PIN_13
#define SDCARD_DETECT_GPIO_Port  GPIOC

/* SD power enable, active-low (PB4) */
#define SD_EN_BAR_Pin        GPIO_PIN_4
#define SD_EN_BAR_GPIO_Port  GPIOB

/* Battery / ADC */
#define VBAT_SCALED_Pin       GPIO_PIN_0   /* PA0 */
#define VBAT_SCALED_GPIO_Port GPIOA
#define VBAT_MONITOR_Pin      GPIO_PIN_5   /* PB5 */
#define VBAT_MONITOR_GPIO_Port GPIOB
#define ADC_EN_Pin            GPIO_PIN_15  /* PB15 */
#define ADC_EN_GPIO_Port      GPIOB

/* VBUS scaled — USB detect (PA15) */
#define VBUS_SCALED_Pin       GPIO_PIN_15
#define VBUS_SCALED_GPIO_Port GPIOA

/* Audio codec (TLV320ADC3120) on I2C1, 7-bit addr 0x4D */
#define CODEC_I2C_ADDR        (0x4D << 1)

/* PS/SYNC for buck-boost (PC2) */
#define PS_SYNC_Pin           GPIO_PIN_2
#define PS_SYNC_GPIO_Port     GPIOC

/* SAI1 pins: PA8=BCLK, PA9=WCLK, PA10=DOUT, PB8=MCLK */
/* I2C1 pins: PB6=SCL, PB7=SDA */
/* USB:       PA11=DM, PA12=DP */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
