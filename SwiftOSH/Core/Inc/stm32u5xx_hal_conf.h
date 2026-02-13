/**
  ******************************************************************************
  * @file    stm32u5xx_hal_conf.h
  * @brief   HAL configuration for SwiftOSH (STM32U545RET6Q)
  ******************************************************************************
  */
#ifndef STM32U5xx_HAL_CONF_H
#define STM32U5xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Module selection ---- */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
/* #define HAL_GPDMA_MODULE_ENABLED */  /* Not available in HAL V1.7.0 — uses DMA instead */
#define HAL_I2C_MODULE_ENABLED
#define HAL_LPTIM_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_RTC_MODULE_ENABLED
#define HAL_SAI_MODULE_ENABLED
#define HAL_SD_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* ---- Oscillator values ---- */
#if !defined(HSE_VALUE)
  #define HSE_VALUE    12288000UL   /* 12.288 MHz crystal */
#endif

#if !defined(HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    100UL
#endif

#if !defined(MSI_VALUE)
  #define MSI_VALUE    4000000UL
#endif

#if !defined(HSI_VALUE)
  #define HSI_VALUE    16000000UL
#endif

#if !defined(HSI48_VALUE)
  #define HSI48_VALUE  48000000UL
#endif

#if !defined(LSE_VALUE)
  #define LSE_VALUE    32768UL
#endif

#if !defined(LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    5000UL
#endif

#if !defined(LSI_VALUE)
  #define LSI_VALUE    32000UL
#endif

#if !defined(EXTERNAL_SAI1_CLOCK_VALUE)
  #define EXTERNAL_SAI1_CLOCK_VALUE  12288000UL
#endif

/* ---- System configuration ---- */
#define VDD_VALUE                    3300UL
#define TICK_INT_PRIORITY            15UL
#define USE_RTOS                     0U
#define PREFETCH_ENABLE              1U
#define INSTRUCTION_CACHE_ENABLE     1U
#define DATA_CACHE_ENABLE            1U

/* Enable SAI register callbacks for HAL_SAI_RegisterCallback() */
#define USE_HAL_SAI_REGISTER_CALLBACKS  1U

/* ---- HAL module includes ---- */
#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32u5xx_hal_rcc.h"
#endif
#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32u5xx_hal_gpio.h"
#endif
#ifdef HAL_GPDMA_MODULE_ENABLED
  #include "stm32u5xx_hal_gpdma.h"
#endif
#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32u5xx_hal_dma.h"
#endif
#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32u5xx_hal_cortex.h"
#endif
#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32u5xx_hal_adc.h"
#endif
#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32u5xx_hal_exti.h"
#endif
#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32u5xx_hal_flash.h"
#endif
#ifdef HAL_I2C_MODULE_ENABLED
  #include "stm32u5xx_hal_i2c.h"
#endif
#ifdef HAL_LPTIM_MODULE_ENABLED
  #include "stm32u5xx_hal_lptim.h"
#endif
#ifdef HAL_PCD_MODULE_ENABLED
  #include "stm32u5xx_hal_pcd.h"
#endif
#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32u5xx_hal_pwr.h"
#endif
#ifdef HAL_RTC_MODULE_ENABLED
  #include "stm32u5xx_hal_rtc.h"
#endif
#ifdef HAL_SAI_MODULE_ENABLED
  #include "stm32u5xx_hal_sai.h"
#endif
#ifdef HAL_SD_MODULE_ENABLED
  #include "stm32u5xx_hal_sd.h"
#endif
#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32u5xx_hal_tim.h"
#endif
#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32u5xx_hal_uart.h"
#endif

/* ---- Assert ---- */
/* #define USE_FULL_ASSERT  1U */
#ifdef USE_FULL_ASSERT
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
  void assert_failed(uint8_t *file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32U5xx_HAL_CONF_H */
