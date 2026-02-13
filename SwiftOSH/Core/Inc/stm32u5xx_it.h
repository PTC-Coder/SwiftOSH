/**
  ******************************************************************************
  * @file    stm32u5xx_it.h
  * @brief   Interrupt handler prototypes for SwiftOSH
  ******************************************************************************
  */
#ifndef __STM32U5xx_IT_H
#define __STM32U5xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);

/* Peripheral interrupts */
void EXTI0_IRQHandler(void);
void EXTI6_IRQHandler(void);
void EXTI13_IRQHandler(void);
void GPDMA1_Channel0_IRQHandler(void);
void TIM6_IRQHandler(void);
void RTC_IRQHandler(void);
void SDMMC1_IRQHandler(void);
void USB_IRQHandler(void);
void SAI1_IRQHandler(void);
void LPTIM1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32U5xx_IT_H */
