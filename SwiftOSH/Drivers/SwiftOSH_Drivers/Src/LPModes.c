/* LPModes.c — SwiftOSH (STM32U545RET6Q)
 * Ported from SwiftOne LPModes.c
 *
 * Key differences from STM32L4R9:
 *   - STM32U5 uses HAL_PWREx_EnterSTOP2Mode() instead of HAL_PWREx_EnterSTOP2Mode()
 *   - No PLLSAI2 — STM32U5 uses PLL2 for SAI1 audio clock
 *   - Wake-up flag clearing uses different register layout
 *   - Voltage scaling API is the same
 *   - Clock tree: HSE 12.288 MHz -> PLL1 ~160 MHz, PLL2 for SAI1
 */

#include "LPModes.h"
#include "stm32u5xx_hal.h"

/**
  * @brief  Enter regular Sleep mode (WFI). ~1-3mA idle.
  *         Used as fallback during init before Stop 2 is safe.
  */
void LPModes_Sleep(void)
{
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_ResumeTick();
}

/**
  * @brief  Enter Stop 2 low-power mode.
  *         HSE, PLL1, PLL2 are lost. SRAM1/2 retained.
  *         RTC, LPTIM1 (LSE) remain active.
  *         Wake-up sources: RTC Alarm (EXTI), LPTIM1 (EXTI), EXTI lines.
  *
  * @note   Caller MUST call LPModes_RestoreClockAfterStop2() after wake-up.
  */
void LPModes_EnterStop2(void)
{
    /* Clear all wake-up flags — STM32U5 uses PWR_WAKEUP_FLAGn (not PWR_FLAG_WUFn) */
    HAL_PWR_EnableBkUpAccess();
    __HAL_PWR_CLEAR_FLAG(PWR_WAKEUP_ALL_FLAG);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SBF);

    HAL_SuspendTick();

    /* Enter Stop 2 — all EXTI-configured interrupts can wake us */
    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
}

/**
  * @brief  Restore system clocks after waking from Stop 2.
  *         On wake-up the MCU runs from MSI at 4MHz.
  *         Re-enables HSE, PLL1, PLL2, switches SYSCLK back.
  */
void LPModes_RestoreClockAfterStop2(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /* Voltage scaling must be set before enabling PLL */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Re-enable HSE and PLL1 (LSE survives Stop 2) */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 3;
    RCC_OscInitStruct.PLL.PLLN = 39;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 1;
    RCC_OscInitStruct.PLL.PLLRGE   = RCC_PLLVCIRANGE_1;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        /* Fall back to MSI — system will limp but not hang */
        HAL_ResumeTick();
        return;
    }

    /* Switch SYSCLK back to PLL1 output */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2  |
                                  RCC_CLOCKTYPE_PCLK3;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        HAL_ResumeTick();
        return;
    }

    /* Re-enable PLL2 for SAI1 audio clock */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
    PeriphClkInit.Sai1ClockSelection   = RCC_SAI1CLKSOURCE_PLL2;
    PeriphClkInit.PLL2.PLL2Source = RCC_PLLSOURCE_HSE;
    PeriphClkInit.PLL2.PLL2M = 3;
    PeriphClkInit.PLL2.PLL2N = 64;
    PeriphClkInit.PLL2.PLL2P = 8;
    PeriphClkInit.PLL2.PLL2Q = 8;
    PeriphClkInit.PLL2.PLL2R = 8;
    PeriphClkInit.PLL2.PLL2RGE   = RCC_PLLVCIRANGE_1;
    PeriphClkInit.PLL2.PLL2FRACN = 0;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    HAL_ResumeTick();
}
