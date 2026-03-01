/* StatusLED.c — SwiftOSH (STM32U545RET6Q)
 * Ported from SwiftOne StatusLED.c
 *
 * SwiftOSH LED polarities (all active-high):
 *   RED   = PC1 (SET=ON, RESET=OFF)
 *   GREEN = PA1 (SET=ON, RESET=OFF)
 *   BLUE  = PA2 (SET=ON, RESET=OFF)
 *
 * On the original SwiftOne, LPTIM2 PWM output was hardware-routed to LED
 * pins via AF14. On STM32U545, the LED pins don't have LPTIM2 AF mapping,
 * so we use LPTIM2 auto-reload interrupt to toggle LEDs in software.
 * This gives the same slow-blink visual effect.
 */

#include "main.h"
#include "StatusLED.h"

#define BLINK_PERIOD  (uint32_t)(1000 - 1)  /* ~3.9s at LSE/128 = 256 Hz */
#define BLINK_ON_TICKS (uint32_t)(38 - 1)  /* ~150ms ON pulse */

LPTIM_HandleTypeDef hlptim2;

/* Which LEDs should blink (bitmask) */
volatile uint8_t led_blink_red;
volatile uint8_t led_blink_grn;
volatile uint8_t led_blink_blu;

void StatusLED_Initialize(void)
{
    __HAL_RCC_LPTIM2_CLK_ENABLE();

    hlptim2.Instance = LPTIM2;
    hlptim2.Init.Clock.Source    = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
    hlptim2.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV128;
    hlptim2.Init.Trigger.Source  = LPTIM_TRIGSOURCE_SOFTWARE;
    hlptim2.Init.Period          = BLINK_PERIOD;
    hlptim2.Init.UpdateMode      = LPTIM_UPDATE_IMMEDIATE;
    hlptim2.Init.CounterSource   = LPTIM_COUNTERSOURCE_INTERNAL;
    hlptim2.Init.Input1Source    = LPTIM_INPUT1SOURCE_GPIO;
    hlptim2.Init.Input2Source    = LPTIM_INPUT2SOURCE_GPIO;
    hlptim2.Init.RepetitionCounter = 0;
    HAL_LPTIM_Init(&hlptim2);

    /* Configure OC channel 1 for compare/polarity */
    LPTIM_OC_ConfigTypeDef sOCConfig = {0};
    sOCConfig.Pulse      = BLINK_ON_TICKS;
    sOCConfig.OCPolarity  = LPTIM_OCPOLARITY_LOW;
    HAL_LPTIM_OC_ConfigChannel(&hlptim2, &sOCConfig, LPTIM_CHANNEL_1);

    HAL_NVIC_SetPriority(LPTIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(LPTIM2_IRQn);

    led_blink_red = 0;
    led_blink_grn = 0;
    led_blink_blu = 0;
}

static void StartBlink(uint8_t red, uint8_t grn, uint8_t blu)
{
    HAL_LPTIM_PWM_Stop(&hlptim2, LPTIM_CHANNEL_1);
    led_blink_red = red;
    led_blink_grn = grn;
    led_blink_blu = blu;

    /* Turn off all LEDs first (all active-high: RESET = OFF) */
    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_RESET);

    /* Start LPTIM2 PWM — the auto-reload interrupt will toggle LEDs */
    HAL_LPTIM_PWM_Start_IT(&hlptim2, LPTIM_CHANNEL_1);
}

void StatusLED_AllOutputs(void)
{
    HAL_LPTIM_PWM_Stop(&hlptim2, LPTIM_CHANNEL_1);
    led_blink_red = 0;
    led_blink_grn = 0;
    led_blink_blu = 0;

    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);   /* RED off */
    HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_RESET);  /* GREEN off */
    HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_RESET);  /* BLUE off */
}

void StatusLED_LowBatteryMode(void)
{
    HAL_LPTIM_PWM_Stop(&hlptim2, LPTIM_CHANNEL_1);
    StatusLED_AllOutputs();
}

void StatusLED_BlinkRedLED(void)
{
    StartBlink(1, 0, 0);
}

void StatusLED_BlinkGreenLED(void)
{
    StartBlink(0, 1, 0);
}

void StatusLED_BlinkBlueLED(void)
{
    StartBlink(0, 0, 1);
}

void StatusLED_BlinkBlueGreenLED(void)
{
    StartBlink(0, 1, 1);
}

void StatusLED_BlinkRedBlueGreenLED(void)
{
    StartBlink(1, 1, 1);
}

void StatusLED_BlinkRedBlueLED(void)
{
    StartBlink(1, 0, 1);
}

void StatusLED_SolidBlueLED(void)
{
    HAL_LPTIM_PWM_Stop(&hlptim2, LPTIM_CHANNEL_1);
    led_blink_red = 0; led_blink_grn = 0; led_blink_blu = 0;
    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);   /* RED off */
    HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_RESET);  /* GREEN off */
    HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_SET);     /* BLUE on */
}

void StatusLED_SolidGreenLED(void)
{
    HAL_LPTIM_PWM_Stop(&hlptim2, LPTIM_CHANNEL_1);
    led_blink_red = 0; led_blink_grn = 0; led_blink_blu = 0;
    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);   /* RED off */
    HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_SET);    /* GREEN on */
    HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_RESET);  /* BLUE off */
}

void StatusLED_SolidAllLED(void)
{
    HAL_LPTIM_PWM_Stop(&hlptim2, LPTIM_CHANNEL_1);
    led_blink_red = 0; led_blink_grn = 0; led_blink_blu = 0;
    HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);     /* RED on (active-high) */
    HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_RESET);   /* GREEN on (active-low) */
    HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_SET);     /* BLUE on (active-high) */
}
