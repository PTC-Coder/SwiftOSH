/**
  ******************************************************************************
  * @file    stm32u5xx_it.c
  * @brief   Interrupt Service Routines — SwiftOSH
  ******************************************************************************
  */
#include "main.h"
#include "stm32u5xx_it.h"

extern PCD_HandleTypeDef   hpcd_USB_DRD_FS;
extern LPTIM_HandleTypeDef hlptim1;
extern RTC_HandleTypeDef   hrtc;
extern DMA_HandleTypeDef   hdma_sai1_a;
extern SAI_HandleTypeDef   hsai_BlockA1;
extern SD_HandleTypeDef    hsd1;
extern TIM_HandleTypeDef   htim6;

/* ---- Cortex-M33 exceptions ---- */
/* NOTE: SVC_Handler, PendSV_Handler, SysTick_Handler are provided by the
   FreeRTOS port via #define mappings in FreeRTOSConfig.h. Do NOT define
   them here — it causes multiple-definition linker errors. */

void NMI_Handler(void)
{
  while (1) {}
}

void HardFault_Handler(void)
{
  /* Blink RED LED so we can see HardFaults during bring-up
     RED (PC1) is active-high: HIGH = on, LOW = off */
  RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOCEN;
  (void)RCC->AHB2ENR1;
  GPIOC->MODER = (GPIOC->MODER & ~(3UL << (1 * 2))) | (1UL << (1 * 2));
  GPIOC->OTYPER &= ~(1UL << 1);
  while (1)
  {
    GPIOC->BSRR = (1UL << 1);         /* PC1 high = RED on */
    for (volatile uint32_t i = 0; i < 200000; i++) {}
    GPIOC->BSRR = (1UL << (1 + 16));  /* PC1 low = RED off */
    for (volatile uint32_t i = 0; i < 200000; i++) {}
  }
}

void MemManage_Handler(void)
{
  while (1) {}
}

void BusFault_Handler(void)
{
  while (1) {}
}

void UsageFault_Handler(void)
{
  while (1) {}
}

void DebugMon_Handler(void)
{
}

/* ---- Peripheral interrupts ---- */

void TIM6_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
}

void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void EXTI6_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
}

void EXTI13_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

void GPDMA1_Channel0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_sai1_a);
}

void RTC_IRQHandler(void)
{
  HAL_RTC_AlarmIRQHandler(&hrtc);
}

void SDMMC1_IRQHandler(void)
{
  HAL_SD_IRQHandler(&hsd1);
}

void USB_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_DRD_FS);
}

void SAI1_IRQHandler(void)
{
  HAL_SAI_IRQHandler(&hsai_BlockA1);
}

void LPTIM1_IRQHandler(void)
{
  HAL_LPTIM_IRQHandler(&hlptim1);
}
extern LPTIM_HandleTypeDef hlptim2;

void LPTIM2_IRQHandler(void)
{
  HAL_LPTIM_IRQHandler(&hlptim2);
}
