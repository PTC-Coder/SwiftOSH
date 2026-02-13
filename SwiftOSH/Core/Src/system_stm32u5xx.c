/**
  ******************************************************************************
  * @file    system_stm32u5xx.c
  * @brief   CMSIS system init — SwiftOSH (STM32U545RET6Q)
  *          Minimal: sets SystemCoreClock, called by startup before main().
  ******************************************************************************
  */
#include "stm32u5xx.h"

#if !defined(HSE_VALUE)
  #define HSE_VALUE  12288000UL
#endif

#if !defined(MSI_VALUE)
  #define MSI_VALUE  4000000UL
#endif

#if !defined(HSI_VALUE)
  #define HSI_VALUE  16000000UL
#endif

uint32_t SystemCoreClock = 4000000UL;  /* Default MSIS at reset */
const uint8_t AHBPrescTable[16] = {0U,0U,0U,0U,0U,0U,0U,0U,
                                    1U,2U,3U,4U,6U,7U,8U,9U};
const uint8_t APBPrescTable[8]  = {0U,0U,0U,0U,1U,2U,3U,4U};
const uint32_t MSIRangeTable[16] = {
  48000000UL, 24000000UL, 16000000UL, 12000000UL,
   8000000UL,  4000000UL,  2000000UL,  1500000UL,
   1000000UL,  3072000UL,  1536000UL,    768000UL,
    400000UL,   200000UL,   150000UL,    100000UL
};

void SystemInit(void)
{
  /* FPU settings — enable CP10 and CP11 */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 20U) | (3UL << 22U));
#endif

  /* Reset RCC clock config to default */
  RCC->CR |= RCC_CR_MSISON;
  RCC->CFGR1 = 0x00000000UL;
  RCC->CFGR2 = 0x00000000UL;
  RCC->CFGR3 = 0x00000000UL;
  RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSION | RCC_CR_HSI48ON |
                RCC_CR_PLL1ON | RCC_CR_PLL2ON | RCC_CR_PLL3ON);
  RCC->PLL1CFGR = 0x00000000UL;
  RCC->PLL2CFGR = 0x00000000UL;
  RCC->PLL3CFGR = 0x00000000UL;
  RCC->CR &= ~RCC_CR_CSSON;
  RCC->CICR = 0xFFFFFFFFUL;  /* Clear all interrupt flags */
}

void SystemCoreClockUpdate(void)
{
  uint32_t pllsource, pllm, pllr, plln, tmp;
  float pllvco;

  switch (RCC->CFGR1 & RCC_CFGR1_SWS)
  {
    case RCC_CFGR1_SWS_0:  /* HSI */
      SystemCoreClock = HSI_VALUE;
      break;
    case RCC_CFGR1_SWS_1:  /* HSE */
      SystemCoreClock = HSE_VALUE;
      break;
    case (RCC_CFGR1_SWS_0 | RCC_CFGR1_SWS_1):  /* PLL */
      pllsource = (RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1SRC);
      pllm = ((RCC->PLL1CFGR & RCC_PLL1CFGR_PLL1M) >> RCC_PLL1CFGR_PLL1M_Pos) + 1U;
      plln = (RCC->PLL1DIVR & RCC_PLL1DIVR_PLL1N) + 1U;
      pllr = ((RCC->PLL1DIVR & RCC_PLL1DIVR_PLL1R) >> RCC_PLL1DIVR_PLL1R_Pos) + 1U;

      switch (pllsource)
      {
        case 0x01: tmp = MSI_VALUE;   break;
        case 0x02: tmp = HSI_VALUE;   break;
        case 0x03: tmp = HSE_VALUE;   break;
        default:   tmp = MSI_VALUE;   break;
      }
      pllvco = ((float)tmp / (float)pllm) * (float)plln;
      SystemCoreClock = (uint32_t)(pllvco / (float)pllr);
      break;
    default:  /* MSI */
    {
      uint32_t msirange;
      if ((RCC->ICSCR1 & RCC_ICSCR1_MSIRGSEL) != 0U)
        msirange = (RCC->ICSCR1 & RCC_ICSCR1_MSISRANGE) >> RCC_ICSCR1_MSISRANGE_Pos;
      else
        msirange = (RCC->CSR & RCC_CSR_MSISSRANGE) >> RCC_CSR_MSISSRANGE_Pos;
      SystemCoreClock = MSIRangeTable[msirange];
      break;
    }
  }

  tmp = AHBPrescTable[((RCC->CFGR2 & RCC_CFGR2_HPRE) >> RCC_CFGR2_HPRE_Pos)];
  SystemCoreClock >>= tmp;
}
