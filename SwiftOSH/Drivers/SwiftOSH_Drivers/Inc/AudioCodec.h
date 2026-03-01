/* AudioCodec.h — SwiftOSH (STM32U545RET6Q) */
#ifndef __AudioCodec_H__
#define __AudioCodec_H__

#include "stm32u5xx_hal.h"

unsigned char AudioCodec_Initialize(I2C_HandleTypeDef *hi2c, uint8_t gain, uint8_t mic_bias_cfg);
unsigned char AudioCodec_PowerUp(I2C_HandleTypeDef *hi2c, uint8_t mic_bias_cfg);
unsigned char AudioCodec_SleepMode(I2C_HandleTypeDef *hi2c);
unsigned char AudioCodec_WakeFromSleep(I2C_HandleTypeDef *hi2c, uint8_t mic_bias_cfg);

#endif /* __AudioCodec_H__ */
