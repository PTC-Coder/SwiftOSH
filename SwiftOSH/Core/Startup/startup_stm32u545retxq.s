/**
  ******************************************************************************
  * @file    startup_stm32u545retxq.s
  * @brief   STM32U545RET6Q vector table + reset handler for GCC
  ******************************************************************************
  */

  .syntax unified
  .cpu cortex-m33
  .fpu fpv5-sp-d16
  .thumb

.global g_pfnVectors
.global Default_Handler

  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0

/* Disable ALL interrupts at CPU level */
  cpsid i

/* Disable SysTick */
  ldr   r0, =0xE000E010
  movs  r1, #0
  str   r1, [r0]

/* Clear any pending PendSV and SysTick from bootloader */
  ldr   r0, =0xE000ED04   /* SCB->ICSR */
  ldr   r1, =0x0A000000   /* PENDSVCLR | PENDSTCLR */
  str   r1, [r0]

/* Call SystemInit before .data/.bss init (resets RCC, enables FPU) */
  bl    SystemInit

/* Copy .data section from flash to SRAM */
  ldr   r0, =_sdata       /* destination start (RAM) */
  ldr   r1, =_edata       /* destination end (RAM) */
  ldr   r2, =_sidata      /* source start (flash) */
  b     CopyDataCheck
CopyDataLoop:
  ldr   r3, [r2], #4
  str   r3, [r0], #4
CopyDataCheck:
  cmp   r0, r1
  blt   CopyDataLoop

/* Zero-fill .bss section */
  ldr   r0, =_sbss        /* start of .bss (RAM) */
  ldr   r1, =_ebss        /* end of .bss (RAM) */
  movs  r2, #0
  b     ZeroBssCheck
ZeroBssLoop:
  str   r2, [r0], #4
ZeroBssCheck:
  cmp   r0, r1
  blt   ZeroBssLoop

/* Re-enable interrupts before entering main */
  cpsie i

  bl    main

LoopForever:
  b     LoopForever

  .size Reset_Handler, .-Reset_Handler

  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b     Infinite_Loop
  .size Default_Handler, .-Default_Handler

/* ---- Vector Table ---- */
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  /* External Interrupts — STM32U545 (first 91 entries) */
  .word WWDG_IRQHandler              /* 0 */
  .word PVD_PVM_IRQHandler           /* 1 */
  .word RTC_IRQHandler               /* 2 */
  .word RTC_S_IRQHandler             /* 3 */
  .word TAMP_IRQHandler              /* 4 */
  .word RAMCFG_IRQHandler            /* 5 */
  .word FLASH_IRQHandler             /* 6 */
  .word FLASH_S_IRQHandler           /* 7 */
  .word GTZC_IRQHandler              /* 8 */
  .word RCC_IRQHandler               /* 9 */
  .word RCC_S_IRQHandler             /* 10 */
  .word EXTI0_IRQHandler             /* 11 */
  .word EXTI1_IRQHandler             /* 12 */
  .word EXTI2_IRQHandler             /* 13 */
  .word EXTI3_IRQHandler             /* 14 */
  .word EXTI4_IRQHandler             /* 15 */
  .word EXTI5_IRQHandler             /* 16 */
  .word EXTI6_IRQHandler             /* 17 */
  .word EXTI7_IRQHandler             /* 18 */
  .word EXTI8_IRQHandler             /* 19 */
  .word EXTI9_IRQHandler             /* 20 */
  .word EXTI10_IRQHandler            /* 21 */
  .word EXTI11_IRQHandler            /* 22 */
  .word EXTI12_IRQHandler            /* 23 */
  .word EXTI13_IRQHandler            /* 24 */
  .word EXTI14_IRQHandler            /* 25 */
  .word EXTI15_IRQHandler            /* 26 */
  .word IWDG_IRQHandler              /* 27 */
  .word 0                            /* 28 reserved */
  .word GPDMA1_Channel0_IRQHandler   /* 29 */
  .word GPDMA1_Channel1_IRQHandler   /* 30 */
  .word GPDMA1_Channel2_IRQHandler   /* 31 */
  .word GPDMA1_Channel3_IRQHandler   /* 32 */
  .word GPDMA1_Channel4_IRQHandler   /* 33 */
  .word GPDMA1_Channel5_IRQHandler   /* 34 */
  .word GPDMA1_Channel6_IRQHandler   /* 35 */
  .word GPDMA1_Channel7_IRQHandler   /* 36 */
  .word ADC1_IRQHandler              /* 37 */
  .word DAC1_IRQHandler              /* 38 */
  .word FDCAN1_IT0_IRQHandler        /* 39 */
  .word FDCAN1_IT1_IRQHandler        /* 40 */
  .word TIM1_BRK_IRQHandler          /* 41 */
  .word TIM1_UP_IRQHandler           /* 42 */
  .word TIM1_TRG_COM_IRQHandler      /* 43 */
  .word TIM1_CC_IRQHandler           /* 44 */
  .word TIM2_IRQHandler              /* 45 */
  .word TIM3_IRQHandler              /* 46 */
  .word TIM4_IRQHandler              /* 47 */
  .word TIM5_IRQHandler              /* 48 */
  .word TIM6_IRQHandler              /* 49 */
  .word TIM7_IRQHandler              /* 50 */
  .word TIM8_BRK_IRQHandler          /* 51 */
  .word TIM8_UP_IRQHandler           /* 52 */
  .word TIM8_TRG_COM_IRQHandler      /* 53 */
  .word TIM8_CC_IRQHandler           /* 54 */
  .word I2C1_EV_IRQHandler           /* 55 */
  .word I2C1_ER_IRQHandler           /* 56 */
  .word I2C2_EV_IRQHandler           /* 57 */
  .word I2C2_ER_IRQHandler           /* 58 */
  .word SPI1_IRQHandler              /* 59 */
  .word SPI2_IRQHandler              /* 60 */
  .word USART1_IRQHandler            /* 61 */
  .word 0                            /* 62 reserved (no USART2 on STM32U545) */
  .word USART3_IRQHandler            /* 63 */
  .word UART4_IRQHandler             /* 64 */
  .word UART5_IRQHandler             /* 65 */
  .word LPUART1_IRQHandler           /* 66 */
  .word LPTIM1_IRQHandler            /* 67 */
  .word LPTIM2_IRQHandler            /* 68 */
  .word TIM15_IRQHandler             /* 69 */
  .word TIM16_IRQHandler             /* 70 */
  .word TIM17_IRQHandler             /* 71 */
  .word COMP_IRQHandler              /* 72 */
  .word USB_IRQHandler               /* 73 */
  .word CRS_IRQHandler               /* 74 */
  .word 0                            /* 75 reserved */
  .word OCTOSPI1_IRQHandler          /* 76 */
  .word PWR_S3WU_IRQHandler          /* 77 */
  .word SDMMC1_IRQHandler            /* 78 */
  .word 0                            /* 78 reserved */
  .word GPDMA1_Channel8_IRQHandler   /* 79 */
  .word GPDMA1_Channel9_IRQHandler   /* 80 */
  .word GPDMA1_Channel10_IRQHandler  /* 81 */
  .word GPDMA1_Channel11_IRQHandler  /* 82 */
  .word 0                            /* 83 reserved */
  .word 0                            /* 84 reserved */
  .word 0                            /* 85 reserved */
  .word ICACHE_IRQHandler            /* 86 */
  .word 0                            /* 87 reserved */
  .word AES_IRQHandler               /* 88 */
  .word HASH_IRQHandler              /* 89 */
  .word RNG_IRQHandler               /* 90 */
  .word 0                            /* 91 reserved */
  .word 0                            /* 92 reserved */
  .word 0                            /* 93 reserved */
  .word SAES_IRQHandler              /* 94 */
  .word I2C3_EV_IRQHandler           /* 95 */
  .word I2C3_ER_IRQHandler           /* 96 */
  .word SAI1_IRQHandler              /* 97 */
  .word 0                            /* 98 reserved */
  .word TSC_IRQHandler               /* 99 */
  .size g_pfnVectors, .-g_pfnVectors

/* ---- Weak aliases for all handlers ---- */
  .weak NMI_Handler
  .thumb_set NMI_Handler, Default_Handler
  .weak HardFault_Handler
  .thumb_set HardFault_Handler, Default_Handler
  .weak MemManage_Handler
  .thumb_set MemManage_Handler, Default_Handler
  .weak BusFault_Handler
  .thumb_set BusFault_Handler, Default_Handler
  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler, Default_Handler
  .weak SVC_Handler
  .thumb_set SVC_Handler, Default_Handler
  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler, Default_Handler
  .weak PendSV_Handler
  .thumb_set PendSV_Handler, Default_Handler
  .weak SysTick_Handler
  .thumb_set SysTick_Handler, Default_Handler
  .weak WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler, Default_Handler
  .weak PVD_PVM_IRQHandler
  .thumb_set PVD_PVM_IRQHandler, Default_Handler
  .weak RTC_IRQHandler
  .thumb_set RTC_IRQHandler, Default_Handler
  .weak RTC_S_IRQHandler
  .thumb_set RTC_S_IRQHandler, Default_Handler
  .weak TAMP_IRQHandler
  .thumb_set TAMP_IRQHandler, Default_Handler
  .weak RAMCFG_IRQHandler
  .thumb_set RAMCFG_IRQHandler, Default_Handler
  .weak FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler, Default_Handler
  .weak FLASH_S_IRQHandler
  .thumb_set FLASH_S_IRQHandler, Default_Handler
  .weak GTZC_IRQHandler
  .thumb_set GTZC_IRQHandler, Default_Handler
  .weak RCC_IRQHandler
  .thumb_set RCC_IRQHandler, Default_Handler
  .weak RCC_S_IRQHandler
  .thumb_set RCC_S_IRQHandler, Default_Handler
  .weak EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler, Default_Handler
  .weak EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler, Default_Handler
  .weak EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler, Default_Handler
  .weak EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler, Default_Handler
  .weak EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler, Default_Handler
  .weak EXTI5_IRQHandler
  .thumb_set EXTI5_IRQHandler, Default_Handler
  .weak EXTI6_IRQHandler
  .thumb_set EXTI6_IRQHandler, Default_Handler
  .weak EXTI7_IRQHandler
  .thumb_set EXTI7_IRQHandler, Default_Handler
  .weak EXTI8_IRQHandler
  .thumb_set EXTI8_IRQHandler, Default_Handler
  .weak EXTI9_IRQHandler
  .thumb_set EXTI9_IRQHandler, Default_Handler
  .weak EXTI10_IRQHandler
  .thumb_set EXTI10_IRQHandler, Default_Handler
  .weak EXTI11_IRQHandler
  .thumb_set EXTI11_IRQHandler, Default_Handler
  .weak EXTI12_IRQHandler
  .thumb_set EXTI12_IRQHandler, Default_Handler
  .weak EXTI13_IRQHandler
  .thumb_set EXTI13_IRQHandler, Default_Handler
  .weak EXTI14_IRQHandler
  .thumb_set EXTI14_IRQHandler, Default_Handler
  .weak EXTI15_IRQHandler
  .thumb_set EXTI15_IRQHandler, Default_Handler
  .weak IWDG_IRQHandler
  .thumb_set IWDG_IRQHandler, Default_Handler
  .weak GPDMA1_Channel0_IRQHandler
  .thumb_set GPDMA1_Channel0_IRQHandler, Default_Handler
  .weak GPDMA1_Channel1_IRQHandler
  .thumb_set GPDMA1_Channel1_IRQHandler, Default_Handler
  .weak GPDMA1_Channel2_IRQHandler
  .thumb_set GPDMA1_Channel2_IRQHandler, Default_Handler
  .weak GPDMA1_Channel3_IRQHandler
  .thumb_set GPDMA1_Channel3_IRQHandler, Default_Handler
  .weak GPDMA1_Channel4_IRQHandler
  .thumb_set GPDMA1_Channel4_IRQHandler, Default_Handler
  .weak GPDMA1_Channel5_IRQHandler
  .thumb_set GPDMA1_Channel5_IRQHandler, Default_Handler
  .weak GPDMA1_Channel6_IRQHandler
  .thumb_set GPDMA1_Channel6_IRQHandler, Default_Handler
  .weak GPDMA1_Channel7_IRQHandler
  .thumb_set GPDMA1_Channel7_IRQHandler, Default_Handler
  .weak ADC1_IRQHandler
  .thumb_set ADC1_IRQHandler, Default_Handler
  .weak DAC1_IRQHandler
  .thumb_set DAC1_IRQHandler, Default_Handler
  .weak FDCAN1_IT0_IRQHandler
  .thumb_set FDCAN1_IT0_IRQHandler, Default_Handler
  .weak FDCAN1_IT1_IRQHandler
  .thumb_set FDCAN1_IT1_IRQHandler, Default_Handler
  .weak TIM1_BRK_IRQHandler
  .thumb_set TIM1_BRK_IRQHandler, Default_Handler
  .weak TIM1_UP_IRQHandler
  .thumb_set TIM1_UP_IRQHandler, Default_Handler
  .weak TIM1_TRG_COM_IRQHandler
  .thumb_set TIM1_TRG_COM_IRQHandler, Default_Handler
  .weak TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler, Default_Handler
  .weak TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler, Default_Handler
  .weak TIM3_IRQHandler
  .thumb_set TIM3_IRQHandler, Default_Handler
  .weak TIM4_IRQHandler
  .thumb_set TIM4_IRQHandler, Default_Handler
  .weak TIM5_IRQHandler
  .thumb_set TIM5_IRQHandler, Default_Handler
  .weak TIM6_IRQHandler
  .thumb_set TIM6_IRQHandler, Default_Handler
  .weak TIM7_IRQHandler
  .thumb_set TIM7_IRQHandler, Default_Handler
  .weak TIM8_BRK_IRQHandler
  .thumb_set TIM8_BRK_IRQHandler, Default_Handler
  .weak TIM8_UP_IRQHandler
  .thumb_set TIM8_UP_IRQHandler, Default_Handler
  .weak TIM8_TRG_COM_IRQHandler
  .thumb_set TIM8_TRG_COM_IRQHandler, Default_Handler
  .weak TIM8_CC_IRQHandler
  .thumb_set TIM8_CC_IRQHandler, Default_Handler
  .weak I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler, Default_Handler
  .weak I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler, Default_Handler
  .weak I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler, Default_Handler
  .weak I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler, Default_Handler
  .weak SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler, Default_Handler
  .weak SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler, Default_Handler
  .weak USART1_IRQHandler
  .thumb_set USART1_IRQHandler, Default_Handler
  .weak USART3_IRQHandler
  .thumb_set USART3_IRQHandler, Default_Handler
  .weak UART4_IRQHandler
  .thumb_set UART4_IRQHandler, Default_Handler
  .weak UART5_IRQHandler
  .thumb_set UART5_IRQHandler, Default_Handler
  .weak LPUART1_IRQHandler
  .thumb_set LPUART1_IRQHandler, Default_Handler
  .weak LPTIM1_IRQHandler
  .thumb_set LPTIM1_IRQHandler, Default_Handler
  .weak LPTIM2_IRQHandler
  .thumb_set LPTIM2_IRQHandler, Default_Handler
  .weak TIM15_IRQHandler
  .thumb_set TIM15_IRQHandler, Default_Handler
  .weak TIM16_IRQHandler
  .thumb_set TIM16_IRQHandler, Default_Handler
  .weak TIM17_IRQHandler
  .thumb_set TIM17_IRQHandler, Default_Handler
  .weak COMP_IRQHandler
  .thumb_set COMP_IRQHandler, Default_Handler
  .weak USB_IRQHandler
  .thumb_set USB_IRQHandler, Default_Handler
  .weak CRS_IRQHandler
  .thumb_set CRS_IRQHandler, Default_Handler
  .weak OCTOSPI1_IRQHandler
  .thumb_set OCTOSPI1_IRQHandler, Default_Handler
  .weak PWR_S3WU_IRQHandler
  .thumb_set PWR_S3WU_IRQHandler, Default_Handler
  .weak SDMMC1_IRQHandler
  .thumb_set SDMMC1_IRQHandler, Default_Handler
  .weak GPDMA1_Channel8_IRQHandler
  .thumb_set GPDMA1_Channel8_IRQHandler, Default_Handler
  .weak GPDMA1_Channel9_IRQHandler
  .thumb_set GPDMA1_Channel9_IRQHandler, Default_Handler
  .weak GPDMA1_Channel10_IRQHandler
  .thumb_set GPDMA1_Channel10_IRQHandler, Default_Handler
  .weak GPDMA1_Channel11_IRQHandler
  .thumb_set GPDMA1_Channel11_IRQHandler, Default_Handler
  .weak ICACHE_IRQHandler
  .thumb_set ICACHE_IRQHandler, Default_Handler
  .weak AES_IRQHandler
  .thumb_set AES_IRQHandler, Default_Handler
  .weak HASH_IRQHandler
  .thumb_set HASH_IRQHandler, Default_Handler
  .weak RNG_IRQHandler
  .thumb_set RNG_IRQHandler, Default_Handler
  .weak SAES_IRQHandler
  .thumb_set SAES_IRQHandler, Default_Handler
  .weak I2C3_EV_IRQHandler
  .thumb_set I2C3_EV_IRQHandler, Default_Handler
  .weak I2C3_ER_IRQHandler
  .thumb_set I2C3_ER_IRQHandler, Default_Handler
  .weak SAI1_IRQHandler
  .thumb_set SAI1_IRQHandler, Default_Handler
  .weak TSC_IRQHandler
  .thumb_set TSC_IRQHandler, Default_Handler
