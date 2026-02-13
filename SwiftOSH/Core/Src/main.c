/**
  ******************************************************************************
  * @file    main.c
  * @brief   SwiftOSH — STM32U545RET6Q port of SwiftOne firmware
  *          FreeRTOS + USB Custom HID + SAI1 audio + SDMMC1 + I2C codec
  *          (SD FW update capability omitted for now)
  ******************************************************************************
  */
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"
#include "AudioCodec.h"
#include "SwiftSettings.h"
#include "GeneralDefines.h"
#include "AudioFiles.h"
#include "RTC_Swift.h"
#include "SwiftErrors.h"
#include "FlashLogging.h"
#include "StatusLED.h"
#include "LPModes.h"
#include "ff.h"
#include <stdio.h>
#include <string.h>

/* ---- Peripheral handles ---- */
I2C_HandleTypeDef   hi2c1;
ADC_HandleTypeDef   hadc1;
LPTIM_HandleTypeDef hlptim1;
SAI_HandleTypeDef   hsai_BlockA1;
DMA_HandleTypeDef   hdma_sai1_a;
SD_HandleTypeDef    hsd1;
RTC_HandleTypeDef   hrtc;
TIM_HandleTypeDef   htim6;

/* ---- FreeRTOS objects ---- */
EventGroupHandle_t  xEventGroup;

osThreadId InitializeFATFSHandle;
osThreadId RecordLoopHandle;
osThreadId StandbyHandle;
osThreadId ErrorHandle;
osThreadId BlinkLEDHandle;
osThreadId InitializeScheduleHandle;
osThreadId VoiceMemoHandle;
osThreadId LowBatteryHandle;

/* ---- Event-group bit definitions (same as original) ---- */
#define ALARM_A_SIGNAL        ( 1 << 1 )
#define BUTTON_SIGNAL         ( 1 << 2 )
#define END_OF_DAY_SIGNAL     ( 1 << 3 )
#define WRITEBUFFER1_SIGNAL   ( 1 << 4 )
#define WRITEBUFFER2_SIGNAL   ( 1 << 5 )
#define HALLEFFECT_SIGNAL     ( 1 << 6 )
#define ERROR_SIGNAL          ( 1 << 7 )
#define LOW_BATTERY_SIGNAL    ( 1 << 8 )
#define NEW_VOICE_MEMO_SIGNAL ( 1 << 9 )
#define RED_LED_SIGNAL        ( 1 << 10 )
#define GRN_LED_SIGNAL        ( 1 << 11 )
#define BLU_LED_SIGNAL        ( 1 << 12 )
#define GRNBLU_LED_SIGNAL     ( 1 << 13 )
#define DST_EVENT_SIGNAL      ( 1 << 14 )
#define REDBLUGRN_LED_SIGNAL  ( 1 << 15 )
#define BLUBLUGRN_LED_SIGNAL  ( 1 << 16 )

/* ---- DMA audio buffer ---- */
#define BUFFER_SIZE       32768
#define BUFFER_SIZE_DIV2  (BUFFER_SIZE / 2)
static uint8_t I2SRxBuffer[BUFFER_SIZE] __attribute__((section(".bss")));

/* ---- Stop-2 gate (same logic as original) ---- */
volatile uint8_t g_initComplete = 0;
volatile uint8_t g_stop2Allowed = 1;

/* ---- Settings / state variables ---- */
WAVFile_Attributes WAVFile;
Swift_Schedule     SwiftSchedule;
SWIFT_ERRORS       SwiftErrors;
DST_Settings       DSTSettings;
unsigned int       BufferWritesPerFile;

/* ---- FatFS file objects (static to save stack in tasks) ---- */
static FIL FsFile;
static FIL VoiceMemoFile;

/* ---- Forward declarations ---- */
void SystemClock_Config(void);
void SystemClock_Config_USB(void);
static void MX_GPIO_Init(void);
static void MX_GPIO_Init_USB(void);
static void MX_GPDMA1_Init(void);
static void MX_ADC1_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SAI1_Init(uint32_t AudioSampleRate);
static void MX_SDMMC1_SD_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);

void InitializeFATTask(void const *argument);
void RecordLoopTask(void const *argument);
void StandbyTask(void const *argument);
void ErrorTask(void const *argument);
void BlinkLEDTask(void const *argument);
void InitializeScheduleTask(void const *argument);
void NewVoiceMemoTask(void const *argument);
void LowBatteryTask(void const *argument);

static void InitializeCodecAndDMA(void);
static void WriteSystemLog(const char *inputString);
static void WriteSystemFlashLogToSD(void);
static void WriteDebugFile(void);

static void AudioTransferComplete(SAI_HandleTypeDef *hsai);
static void AudioTransferHalfComplete(SAI_HandleTypeDef *hsai);

float GetBatteryVoltage(void);

/* ================================================================== */
/*                     CLOCK CONFIGURATION                            */
/* ================================================================== */

/**
  * @brief  System clock: HSE 12.288 MHz -> PLL -> 160 MHz SYSCLK
  *         LSE 32.768 kHz for RTC / LPTIM
  *         PLLSAI1 for SAI1 audio clock
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /* Supply configuration — LDO (default for STM32U545) */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    Error_Handler();

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /* HSE 12.288 MHz + LSE 32.768 kHz + PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState  = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  /* 12.288 MHz / 3 = 4.096 MHz VCO input
     4.096 MHz * 39 = 159.744 MHz ~ 160 MHz */
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 39;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;  /* SYSCLK = ~160 MHz */
  RCC_OscInitStruct.PLL.PLLRGE    = RCC_PLLVCIRANGE_1;  /* 4-8 MHz */
  RCC_OscInitStruct.PLL.PLLFRACN  = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  /* HCLK, APB1, APB2, APB3 */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2  |
                                RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    Error_Handler();

  /* Peripheral clocks: RTC=LSE, LPTIM1=LSE, I2C1=PCLK1, SAI1=PLL2,
     SDMMC1=PLL1-P, ADC=SYSCLK */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC  | RCC_PERIPHCLK_LPTIM1 |
                                       RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_SAI1   |
                                       RCC_PERIPHCLK_SDMMC | RCC_PERIPHCLK_ADCDAC;
  PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
  PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.SdmmcClockSelection  = RCC_SDMMCCLKSOURCE_PLL1;
  PeriphClkInit.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_SYSCLK;

  /* PLL2 for SAI1: 12.288 MHz / 3 * 64 / 8 = 32.768 MHz SAI clock
     Gives exact 32 kHz / 48 kHz sample rates */
  PeriphClkInit.Sai1ClockSelection   = RCC_SAI1CLKSOURCE_PLL2;
  PeriphClkInit.PLL2.PLL2Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLL2.PLL2M = 3;
  PeriphClkInit.PLL2.PLL2N = 64;
  PeriphClkInit.PLL2.PLL2P = 8;
  PeriphClkInit.PLL2.PLL2Q = 8;
  PeriphClkInit.PLL2.PLL2R = 8;
  PeriphClkInit.PLL2.PLL2RGE    = RCC_PLLVCIRANGE_1;
  PeriphClkInit.PLL2.PLL2FRACN  = 0;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    Error_Handler();
}

/**
  * @brief  USB clock config: HSI48 for USB, rest same as normal
  */
void SystemClock_Config_USB(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    Error_Handler();

  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 |
                                     RCC_OSCILLATORTYPE_HSE   |
                                     RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState    = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState    = RCC_LSE_ON;
  RCC_OscInitStruct.HSI48State  = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState  = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 39;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE   = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2  |
                                RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    Error_Handler();

  /* USB from HSI48 (via ICLK), RTC from LSE, ADC from SYSCLK */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ICLK | RCC_PERIPHCLK_RTC |
                                       RCC_PERIPHCLK_ADCDAC;
  PeriphClkInit.IclkClockSelection   = RCC_ICLK_CLKSOURCE_HSI48;
  PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    Error_Handler();

  /* CRS: trim HSI48 from LSE */
  __HAL_RCC_CRS_CLK_ENABLE();
  RCC_CRSInitStruct.Prescaler   = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source      = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity    = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;
  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/* ================================================================== */
/*                   PERIPHERAL INIT FUNCTIONS                        */
/* ================================================================== */

static void MX_RTC_Init(void)
{
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv   = 127;
  hrtc.Init.SynchPrediv    = 255;
  hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap    = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.BinMode        = RTC_BINARY_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
    Error_Handler();
}

static void MX_TIM6_Init(void)
{
  /* TIM6 as HAL timebase (1 kHz) so SysTick is free for FreeRTOS */
  __HAL_RCC_TIM6_CLK_ENABLE();
  htim6.Instance = TIM6;
  htim6.Init.Prescaler         = (HAL_RCC_GetPCLK1Freq() / 1000000) - 1;
  htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim6.Init.Period            = 999;  /* 1 ms */
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
    Error_Handler();
  HAL_NVIC_SetPriority(TIM6_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(TIM6_IRQn);
  HAL_TIM_Base_Start_IT(&htim6);
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait     = ENABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun               = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode      = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
    Error_Handler();

  /* PA0 = VBAT_SCALED -> ADC1_IN5 on STM32U545 */
  sConfig.Channel      = ADC_CHANNEL_5;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_814CYCLES;
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance              = I2C1;
  hi2c1.Init.Timing           = 0x10707DBC;  /* ~100 kHz at 80 MHz PCLK1 */
  hi2c1.Init.OwnAddress1      = 0;
  hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2      = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
    Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    Error_Handler();
}

static void MX_LPTIM1_Init(void)
{
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source    = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source     = LPTIM_TRIGSOURCE_0;
  hlptim1.Init.Trigger.ActiveEdge = LPTIM_ACTIVEEDGE_FALLING;
  hlptim1.Init.Trigger.SampleTime = LPTIM_TRIGSAMPLETIME_8TRANSITIONS;
  hlptim1.Init.Period          = 0xE000;
  hlptim1.Init.UpdateMode      = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource   = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source    = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source    = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
    Error_Handler();
}

static void MX_SAI1_Init(uint32_t AudioSampleRate)
{
  /* SAI1 Block A — master receiver, I2S-like, 16-bit stereo
     PA8=BCLK, PA9=WCLK, PA10=DOUT(SD_A), PB8=MCLK */
  hsai_BlockA1.Instance            = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol       = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode      = SAI_MODEMASTER_RX;
  hsai_BlockA1.Init.DataSize       = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit       = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing  = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockA1.Init.Synchro        = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive    = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
  hsai_BlockA1.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = AudioSampleRate;
  hsai_BlockA1.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.PdmInit.Activation  = DISABLE;
  hsai_BlockA1.Init.PdmInit.MicPairsNbr = 0;
  hsai_BlockA1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;
  hsai_BlockA1.FrameInit.FrameLength       = 64;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 32;
  hsai_BlockA1.FrameInit.FSDefinition      = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset          = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 1;
  hsai_BlockA1.SlotInit.SlotSize       = SAI_SLOTSIZE_32B;
  hsai_BlockA1.SlotInit.SlotNumber     = 2;
  hsai_BlockA1.SlotInit.SlotActive     = 0x00000002;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
    Error_Handler();
}

static void MX_SDMMC1_SD_Init(void)
{
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide             = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  hsd1.Init.ClockDiv            = 0;
}

static void MX_GPDMA1_Init(void)
{
  __HAL_RCC_GPDMA1_CLK_ENABLE();
  HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
}

/* ================================================================== */
/*                        GPIO INIT                                   */
/* ================================================================== */

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* LEDs — push-pull outputs, default OFF
     RED (PC1) is active-high: LOW = off
     GREEN (PA1) is active-low: HIGH = off
     BLUE (PA2) is active-high: LOW = off */
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);  /* RED off (active-high) */
  HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_SET);    /* GREEN off (active-low) */
  HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_RESET);  /* BLUE off (active-high) */

  GPIO_InitStruct.Pin   = RED_LED_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RED_LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GRN_LED_Pin;
  HAL_GPIO_Init(GRN_LED_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BLU_LED_Pin;
  HAL_GPIO_Init(BLU_LED_GPIO_Port, &GPIO_InitStruct);

  /* SD_EN_bar (PB4) — active-low, drive low to enable SD */
  HAL_GPIO_WritePin(SD_EN_BAR_GPIO_Port, SD_EN_BAR_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin  = SD_EN_BAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(SD_EN_BAR_GPIO_Port, &GPIO_InitStruct);

  /* ADC_EN (PB15) — enable ADC power */
  HAL_GPIO_WritePin(ADC_EN_GPIO_Port, ADC_EN_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin  = ADC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(ADC_EN_GPIO_Port, &GPIO_InitStruct);

  /* PS/SYNC (PC2) — low for power-save mode */
  HAL_GPIO_WritePin(PS_SYNC_GPIO_Port, PS_SYNC_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin  = PS_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PS_SYNC_GPIO_Port, &GPIO_InitStruct);

  /* Hall-effect switch (PC0) — rising-edge EXTI */
  GPIO_InitStruct.Pin  = HALL_EFFECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HALL_EFFECT_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* SD card detect (PC13) — rising/falling EXTI */
  GPIO_InitStruct.Pin  = SDCARD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDCARD_DETECT_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI13_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI13_IRQn);

  /* VBUS_SCALED (PA15) — USB detect */
  GPIO_InitStruct.Pin  = VBUS_SCALED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_SCALED_GPIO_Port, &GPIO_InitStruct);

  /* VBAT_SCALED (PA0) — analog input for ADC */
  GPIO_InitStruct.Pin  = VBAT_SCALED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBAT_SCALED_GPIO_Port, &GPIO_InitStruct);
}

static void MX_GPIO_Init_USB(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Blue LED solid on when USB connected */
  GPIO_InitStruct.Pin   = BLU_LED_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLU_LED_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_SET);  /* Try SET */
}

/* ================================================================== */
/*                     BATTERY VOLTAGE                                */
/* ================================================================== */

float GetBatteryVoltage(void)
{
  float battery_v = -1.0f;
  MX_ADC1_Init();

  uint16_t raw = 0;
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
  {
    raw = HAL_ADC_GetValue(&hadc1);
    /* Scale: 12-bit ADC, 3.3V ref, resistor divider ratio ~3.778 */
    battery_v = (raw * 3.3f / 4095.0f) * 3.778f;
  }
  HAL_ADC_DeInit(&hadc1);
  return battery_v;
}

/* ================================================================== */
/*                     SAI DMA CALLBACKS                              */
/* ================================================================== */

static void AudioTransferComplete(SAI_HandleTypeDef *hsai)
{
  (void)hsai;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xEventGroupSetBitsFromISR(xEventGroup, WRITEBUFFER2_SIGNAL, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void AudioTransferHalfComplete(SAI_HandleTypeDef *hsai)
{
  (void)hsai;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xEventGroupSetBitsFromISR(xEventGroup, WRITEBUFFER1_SIGNAL, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ================================================================== */
/*                     EXTI / RTC CALLBACKS                           */
/* ================================================================== */

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == HALL_EFFECT_Pin)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(xEventGroup, NEW_VOICE_MEMO_SIGNAL, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtclock)
{
  (void)hrtclock;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xEventGroupSetBitsFromISR(xEventGroup, ALARM_A_SIGNAL, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtclock)
{
  (void)hrtclock;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  /* Check battery voltage at midnight */
  float batteryVoltage = GetBatteryVoltage();
  if (batteryVoltage <= 3.0f)
  {
    xEventGroupSetBitsFromISR(xEventGroup, LOW_BATTERY_SIGNAL, &xHigherPriorityTaskWoken);
  }

  /* DST day-of-year tracking */
  DSTSettings.Current_DOY++;
  if (DSTSettings.Current_DOY > DSTSettings.TotalDaysInYear)
    DSTSettings.Current_DOY = 1;

  if (DSTSettings.DST_Active_Flag)
  {
    if (DSTSettings.Its_2AM_Flag == 1)
    {
      DSTSettings.Its_2AM_Flag = 0;
      DSTSettings.Current_DOY--;
      xEventGroupSetBitsFromISR(xEventGroup, DST_EVENT_SIGNAL, &xHigherPriorityTaskWoken);
    }
    else if ((DSTSettings.Current_DOY == DSTSettings.DST_Start_DOY) ||
             (DSTSettings.Current_DOY == DSTSettings.DST_Stop_DOY))
    {
      RTC_EnableAlarmB(SET_ALARMB_2AM);
      DSTSettings.Its_2AM_Flag = 1;
      xEventGroupSetBitsFromISR(xEventGroup, END_OF_DAY_SIGNAL, &xHigherPriorityTaskWoken);
    }
    else
    {
      xEventGroupSetBitsFromISR(xEventGroup, END_OF_DAY_SIGNAL, &xHigherPriorityTaskWoken);
    }
  }
  else
  {
    xEventGroupSetBitsFromISR(xEventGroup, END_OF_DAY_SIGNAL, &xHigherPriorityTaskWoken);
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  if (hlptim->Instance == LPTIM1)
  {
    /* Long-press: only accept if button still held (PC6 low) */
    if (HAL_GPIO_ReadPin(PUSHBUTTON_GPIO_Port, PUSHBUTTON_Pin) == GPIO_PIN_RESET)
    {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xEventGroupSetBitsFromISR(xEventGroup, BUTTON_SIGNAL, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
  else if (hlptim->Instance == LPTIM2)
  {
    /* StatusLED blink toggle — called from LPTIM2 IRQ */
    extern volatile uint8_t led_blink_red, led_blink_grn, led_blink_blu;
    if (led_blink_red)
      HAL_GPIO_TogglePin(RED_LED_GPIO_Port, RED_LED_Pin);
    if (led_blink_grn)
      HAL_GPIO_TogglePin(GRN_LED_GPIO_Port, GRN_LED_Pin);
    if (led_blink_blu)
      HAL_GPIO_TogglePin(BLU_LED_GPIO_Port, BLU_LED_Pin);
  }
}

/* ================================================================== */
/*                     TICKLESS IDLE HOOKS                            */
/* ================================================================== */

void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
  *ulExpectedIdleTime = 0;
  if (g_initComplete && g_stop2Allowed)
  {
    /* Enter Stop 2 — deep low-power, RTC+LPTIM alive on LSE */
    LPModes_EnterStop2();
  }
  else
  {
    /* Regular Sleep during boot or while DMA/recording is active */
    HAL_SuspendTick();
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  }
}

void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
  (void)ulExpectedIdleTime;
  if (g_initComplete && g_stop2Allowed)
  {
    /* Restore clocks after Stop 2 wake-up */
    LPModes_RestoreClockAfterStop2();
  }
  else
  {
    HAL_ResumeTick();
  }
}

/* ================================================================== */
/*                     FreeRTOS TASKS (skeleton)                      */
/* ================================================================== */

/**
  * @brief  FAT/init task — mounts SD, creates child tasks, starts schedule
  */
void InitializeFATTask(void const *argument)
{
  (void)argument;

  WriteSystemLog("** FAT Task Running **");

  /* Create LED blink task */
  osThreadDef(BlinkLED, BlinkLEDTask, osPriorityAboveNormal, 0, 128);
  BlinkLEDHandle = osThreadCreate(osThread(BlinkLED), NULL);

  /* Create error task (suspended) */
  osThreadDef(Error, ErrorTask, osPriorityBelowNormal, 0, 128);
  ErrorHandle = osThreadCreate(osThread(Error), NULL);
  osThreadSuspend(ErrorHandle);

  /* Create low battery task (suspended) */
  osThreadDef(LowBattery, LowBatteryTask, osPriorityBelowNormal, 0, 256);
  LowBatteryHandle = osThreadCreate(osThread(LowBattery), NULL);
  osThreadSuspend(LowBatteryHandle);

  /* Init SDMMC and mount FatFS */
  MX_SDMMC1_SD_Init();
  if (AudioFiles_MountSDCard(&SwiftErrors) != 0)
  {
    WriteSystemLog("<FAT Task> Error mounting SD Card.");
    osThreadResume(ErrorHandle);
    xEventGroupSetBits(xEventGroup, ERROR_SIGNAL);
    osThreadTerminate(NULL);
  }
  WriteSystemLog("<FAT Task> SD Card mounted.");

  /* Write debug file */
  WriteDebugFile();

  /* Dump flash logs to SD, then erase flash log area */
  WriteSystemFlashLogToSD();

  /* Check battery voltage before proceeding */
  {
    float batt = GetBatteryVoltage();
    if (batt > 0.0f && batt < 3.0f)
    {
      WriteSystemLog("<FAT Task> Start Low Battery Task.");
      osThreadResume(LowBatteryHandle);
      xEventGroupSetBits(xEventGroup, LOW_BATTERY_SIGNAL);
      osThreadTerminate(NULL);
    }
  }

  /* Create record-loop task (suspended) */
  osThreadDef(RecordLoop, RecordLoopTask, osPriorityHigh, 0, 1024);
  RecordLoopHandle = osThreadCreate(osThread(RecordLoop), NULL);
  osThreadSuspend(RecordLoopHandle);

  /* Create standby task (suspended) */
  osThreadDef(Standby, StandbyTask, osPriorityAboveNormal, 0, 256);
  StandbyHandle = osThreadCreate(osThread(Standby), NULL);
  osThreadSuspend(StandbyHandle);

  /* Init codec over I2C */
  MX_I2C1_Init();
  AudioCodec_Initialize(&hi2c1);
  AudioCodec_SleepMode(&hi2c1);
  __HAL_RCC_I2C1_CLK_DISABLE();

  /* Read settings from flash */
  SwiftSettings_GetWAVFileAttributes(&WAVFile);
  SwiftSettings_GetSwiftSchedule(&SwiftSchedule);
  SwiftSettings_GetDSTSettings(&DSTSettings);

  /* Initialize DST */
  RTC_InitiateDST(&DSTSettings);

  /* Initialize StatusLED */
  StatusLED_Initialize();

  /* Create and start InitializeScheduleTask */
  osThreadDef(InitSchedule, InitializeScheduleTask, osPriorityNormal, 0, 256);
  InitializeScheduleHandle = osThreadCreate(osThread(InitSchedule), NULL);

  g_initComplete = 1;
  g_stop2Allowed = 0;

  /* Clear error retry counter on successful boot */
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, 0);

  WriteSystemLog("<FAT Task> Init complete, Stop 2 enabled");
  osThreadTerminate(NULL);
}

/**
  * @brief  Record loop — waits for DMA half/full signals, writes to SD.
  *         Full port from SwiftOne including DST, voice memo, END_OF_DAY.
  */
void RecordLoopTask(void const *argument)
{
  (void)argument;
  EventBits_t uxBits;
  UINT byteswritten;
  FRESULT FatFsResult;

  for (;;)
  {
    __HAL_RCC_SDMMC1_CLK_DISABLE();
    uxBits = xEventGroupWaitBits(xEventGroup,
               WRITEBUFFER1_SIGNAL | WRITEBUFFER2_SIGNAL |
               BUTTON_SIGNAL | ALARM_A_SIGNAL | END_OF_DAY_SIGNAL |
               DST_EVENT_SIGNAL | NEW_VOICE_MEMO_SIGNAL |
               LOW_BATTERY_SIGNAL | ERROR_SIGNAL,
               pdTRUE, pdFALSE, osWaitForever);
    __HAL_RCC_SDMMC1_CLK_ENABLE();

    if (uxBits & LOW_BATTERY_SIGNAL)
    {
      /* Emergency stop — halt DMA, close file */
      HAL_SAI_DMAStop(&hsai_BlockA1);
      __HAL_SAI_DISABLE(&hsai_BlockA1);
      HAL_SAI_MspDeInit(&hsai_BlockA1);
      hsai_BlockA1.State = HAL_SAI_STATE_RESET;

      RTC_DeactivateAlarm(RTC_ALARM_A);
      RTC_DeactivateAlarm(RTC_ALARM_B);
      BufferWritesPerFile = WAVFile.FileSize;

      AudioFiles_WriteHeader(&SwiftErrors, &FsFile);
      AudioFiles_CloseFile(&SwiftErrors, &FsFile);

      osThreadResume(LowBatteryHandle);
      osThreadSuspend(NULL);
    }
    else if (uxBits & WRITEBUFFER1_SIGNAL)
    {
      FatFsResult = f_write(&FsFile, &I2SRxBuffer[0], (UINT)(BUFFER_SIZE_DIV2), (void *)&byteswritten);
      if (FatFsResult != FR_OK)
      {
        WriteSystemLog("<RecordLoop> Buffer1 write error");
        xEventGroupSetBits(xEventGroup, ERROR_SIGNAL);
        osThreadTerminate(NULL);
      }
    }
    else if (uxBits & WRITEBUFFER2_SIGNAL)
    {
      FatFsResult = f_write(&FsFile, &I2SRxBuffer[BUFFER_SIZE_DIV2], (UINT)(BUFFER_SIZE_DIV2), (void *)&byteswritten);
      if (FatFsResult != FR_OK)
      {
        WriteSystemLog("<RecordLoop> Buffer2 write error");
        xEventGroupSetBits(xEventGroup, ERROR_SIGNAL);
        osThreadTerminate(NULL);
      }

      if (--BufferWritesPerFile == 0)
      {
        AudioFiles_WriteHeader(&SwiftErrors, &FsFile);
        AudioFiles_CloseFile(&SwiftErrors, &FsFile);
        AudioFiles_NewAudioFile(WAVFile.Filename, DSTSettings.DST_Suffix_To_Use, &FsFile);
        BufferWritesPerFile = WAVFile.FileSize;
      }
    }
    else if (uxBits & (BUTTON_SIGNAL | NEW_VOICE_MEMO_SIGNAL | DST_EVENT_SIGNAL))
    {
      /* Stop DMA, close file */
      HAL_SAI_DMAStop(&hsai_BlockA1);
      __HAL_SAI_DISABLE(&hsai_BlockA1);
      HAL_SAI_MspDeInit(&hsai_BlockA1);
      hsai_BlockA1.State = HAL_SAI_STATE_RESET;

      RTC_DeactivateAlarm(RTC_ALARM_A);
      RTC_DeactivateAlarm(RTC_ALARM_B);
      BufferWritesPerFile = WAVFile.FileSize;

      AudioFiles_WriteHeader(&SwiftErrors, &FsFile);
      AudioFiles_CloseFile(&SwiftErrors, &FsFile);

      if (uxBits & BUTTON_SIGNAL)
      {
        /* Button press during recording — stop and go to standby */
        SwiftSchedule.StartDate.Year = 0;
        SwiftSchedule.StartDate.Date = 1;
        SwiftSchedule.StartDate.Month = 1;
        SwiftSchedule.StopDate.Year = 1;
        SwiftSchedule.StopDate.Date = 1;
        SwiftSchedule.StopDate.Month = 1;
        WAVFile.RecordingDirFlag = 0;

        xEventGroupSetBits(xEventGroup, BLU_LED_SIGNAL);
        __HAL_RCC_I2C1_CLK_ENABLE();
        AudioCodec_SleepMode(&hi2c1);
        __HAL_RCC_I2C1_CLK_DISABLE();

        osThreadResume(StandbyHandle);
      }
      else if (uxBits & NEW_VOICE_MEMO_SIGNAL)
      {
        osThreadDef(VoiceMemo, NewVoiceMemoTask, osPriorityNormal, 0, 512);
        VoiceMemoHandle = osThreadCreate(osThread(VoiceMemo), NULL);
      }
      else if (uxBits & DST_EVENT_SIGNAL)
      {
        RTC_InitiateDST(&DSTSettings);
        osThreadResume(InitializeScheduleHandle);
      }
      osThreadSuspend(NULL);
    }
    else if (uxBits & END_OF_DAY_SIGNAL)
    {
      /* Midnight crossing during recording */
      AudioFiles_WriteHeader(&SwiftErrors, &FsFile);
      AudioFiles_CloseFile(&SwiftErrors, &FsFile);
      BufferWritesPerFile = WAVFile.FileSize;

      if ((SwiftSchedule.StartsAtMidnightFlag) &&
          (RTC_CheckIfBetweenDates(SwiftSchedule.StartDate, SwiftSchedule.StopDate)))
      {
        /* Continue recording into new day directory */
        f_chdir((char const *)WAVFile.RecordingDirName);
        AudioFiles_NewDayDirectory(WAVFile.Filename);
        AudioFiles_NewAudioFile(WAVFile.Filename, DSTSettings.DST_Suffix_To_Use, &FsFile);
        SwiftSchedule.CurrentRecordNumber = 0;
        SetNextStopTime(&SwiftSchedule);
      }
      else
      {
        /* Stop recording, go to standby */
        HAL_SAI_DMAStop(&hsai_BlockA1);
        __HAL_SAI_DISABLE(&hsai_BlockA1);
        HAL_SAI_MspDeInit(&hsai_BlockA1);
        hsai_BlockA1.State = HAL_SAI_STATE_RESET;

        SetNextStartTime(&SwiftSchedule);

        xEventGroupSetBits(xEventGroup, BLU_LED_SIGNAL);
        __HAL_RCC_I2C1_CLK_ENABLE();
        AudioCodec_SleepMode(&hi2c1);
        __HAL_RCC_I2C1_CLK_DISABLE();

        osThreadResume(StandbyHandle);
        osThreadSuspend(NULL);
      }
    }
    else if (uxBits & ALARM_A_SIGNAL)
    {
      /* Stop time alarm — close file, go to standby */
      HAL_SAI_DMAStop(&hsai_BlockA1);
      __HAL_SAI_DISABLE(&hsai_BlockA1);
      HAL_SAI_MspDeInit(&hsai_BlockA1);
      hsai_BlockA1.State = HAL_SAI_STATE_RESET;

      AudioFiles_WriteHeader(&SwiftErrors, &FsFile);
      AudioFiles_CloseFile(&SwiftErrors, &FsFile);
      BufferWritesPerFile = WAVFile.FileSize;

      if ((SwiftSchedule.CurrentRecordNumber + 1) != SwiftSchedule.NumberOfRecords)
        SetNextStartTime(&SwiftSchedule);

      xEventGroupSetBits(xEventGroup, BLU_LED_SIGNAL);
      __HAL_RCC_I2C1_CLK_ENABLE();
      AudioCodec_SleepMode(&hi2c1);
      __HAL_RCC_I2C1_CLK_DISABLE();

      osThreadResume(StandbyHandle);
      osThreadSuspend(NULL);
    }
  }
}

/**
  * @brief  Standby task — waits for alarm/button, then starts recording.
  *         Full port from SwiftOne including END_OF_DAY, DST, voice memo.
  */
void StandbyTask(void const *argument)
{
  (void)argument;
  EventBits_t uxBits;

  for (;;)
  {
    g_stop2Allowed = 1;
    WriteSystemLog("<Standby> Entering Stop 2 standby");
    __HAL_RCC_SDMMC1_CLK_DISABLE();

    uxBits = xEventGroupWaitBits(xEventGroup,
               BUTTON_SIGNAL | ALARM_A_SIGNAL | END_OF_DAY_SIGNAL |
               DST_EVENT_SIGNAL | NEW_VOICE_MEMO_SIGNAL | LOW_BATTERY_SIGNAL,
               pdTRUE, pdFALSE, osWaitForever);

    g_stop2Allowed = 0;
    WriteSystemLog("<Standby> Woke from Stop 2");
    BufferWritesPerFile = WAVFile.FileSize;
    __HAL_RCC_SDMMC1_CLK_ENABLE();

    if (uxBits & LOW_BATTERY_SIGNAL)
    {
      osThreadResume(LowBatteryHandle);
      osThreadSuspend(NULL);
    }
    else if (uxBits & END_OF_DAY_SIGNAL)
    {
      uint8_t returnFlag = RTC_CheckIfBetweenDates(SwiftSchedule.StartDate, SwiftSchedule.StopDate);

      if (returnFlag == 1)
      {
        AudioFiles_NewRecordingDirectory(&WAVFile);
        AudioFiles_NewDayDirectory(WAVFile.Filename);
        WAVFile.DirectoryLevel = 2;

        if (SwiftSchedule.StartsAtMidnightFlag == 1)
        {
          StatusLED_SolidGreenLED();
          AudioFiles_NewAudioFile(WAVFile.Filename, DSTSettings.DST_Suffix_To_Use, &FsFile);
          SwiftSchedule.CurrentRecordNumber = 0;
          SetNextStopTime(&SwiftSchedule);
          InitializeCodecAndDMA();
          xEventGroupSetBits(xEventGroup, GRN_LED_SIGNAL);
          osThreadResume(RecordLoopHandle);
          osThreadSuspend(NULL);
        }
        else
        {
          SwiftSchedule.CurrentRecordNumber = SwiftSchedule.NumberOfRecords;
          SetNextStartTime(&SwiftSchedule);
          /* Loop back to xEventGroupWaitBits to catch ALARM_A_SIGNAL */
        }
      }
      else if (returnFlag == 2)
      {
        SwiftSchedule.CurrentRecordNumber = 0;
        SetNextStartTime(&SwiftSchedule);
        RTC_DeactivateAlarm(RTC_ALARM_A);
        RTC_DeactivateAlarm(RTC_ALARM_B);
      }
    }
    else if (uxBits & BUTTON_SIGNAL)
    {
      StatusLED_SolidGreenLED();
      AudioFiles_NewRecordingDirectory(&WAVFile);
      AudioFiles_NewDayDirectory(WAVFile.Filename);
      WAVFile.DirectoryLevel = 2;
      AudioFiles_NewAudioFile(WAVFile.Filename, DSTSettings.DST_Suffix_To_Use, &FsFile);

      /* Override schedule to continuous */
      SwiftSchedule.EndsAtMidnightFlag = 0;
      SwiftSchedule.StartsAtMidnightFlag = 1;
      SwiftSchedule.CurrentRecordNumber = 0;
      SwiftSchedule.NumberOfRecords = 1;
      SwiftSchedule.ScheduleType = SCHEDULE_TYPE_CONTINUOUS;
      SwiftSchedule.StartDate.Year = 0;
      SwiftSchedule.StartDate.Date = 1;
      SwiftSchedule.StartDate.Month = 1;
      SwiftSchedule.StopDate.Year = 99;
      SwiftSchedule.StopDate.Date = 1;
      SwiftSchedule.StopDate.Month = 1;
      SwiftSchedule.StartStopTimes[0].RecordingStartTime.Hours = 0x00;
      SwiftSchedule.StartStopTimes[0].RecordingStartTime.Minutes = 0x00;
      SwiftSchedule.StartStopTimes[0].RecordingStopTime.Hours = 0x23;
      SwiftSchedule.StartStopTimes[0].RecordingStopTime.Minutes = 0x59;

      SetNextStopTime(&SwiftSchedule);
      RTC_EnableAlarmB(SET_ALARMB_MIDNIGHT);

      InitializeCodecAndDMA();
      xEventGroupSetBits(xEventGroup, GRN_LED_SIGNAL);
      osThreadResume(RecordLoopHandle);
      osThreadSuspend(NULL);
    }
    else if (uxBits & ALARM_A_SIGNAL)
    {
      StatusLED_SolidGreenLED();
      AudioFiles_NewAudioFile(WAVFile.Filename, DSTSettings.DST_Suffix_To_Use, &FsFile);
      SetNextStopTime(&SwiftSchedule);
      xEventGroupSetBits(xEventGroup, GRN_LED_SIGNAL);
      InitializeCodecAndDMA();
      osThreadResume(RecordLoopHandle);
      osThreadSuspend(NULL);
    }
    else if (uxBits & DST_EVENT_SIGNAL)
    {
      RTC_InitiateDST(&DSTSettings);
      osThreadResume(InitializeScheduleHandle);
      osThreadSuspend(NULL);
    }
    else if (uxBits & NEW_VOICE_MEMO_SIGNAL)
    {
      osThreadDef(VoiceMemo, NewVoiceMemoTask, osPriorityNormal, 0, 512);
      VoiceMemoHandle = osThreadCreate(osThread(VoiceMemo), NULL);
      osThreadSuspend(NULL);
    }
  }
}

/**
  * @brief  Error task — blinks red LED on error
  */
void ErrorTask(void const *argument)
{
  (void)argument;
  for (;;)
  {
    xEventGroupWaitBits(xEventGroup, ERROR_SIGNAL, pdTRUE, pdTRUE, osWaitForever);
    xEventGroupSetBits(xEventGroup, RED_LED_SIGNAL);
  }
}

/**
  * @brief  LED blink task — waits for LED signals, blinks accordingly.
  *         Full port from SwiftOne with StatusLED integration.
  */
void BlinkLEDTask(void const *argument)
{
  (void)argument;
  EventBits_t uxBits;
  uint32_t i;

  for (;;)
  {
    uxBits = xEventGroupWaitBits(xEventGroup,
               RED_LED_SIGNAL | GRN_LED_SIGNAL | BLU_LED_SIGNAL |
               GRNBLU_LED_SIGNAL | REDBLUGRN_LED_SIGNAL | BLUBLUGRN_LED_SIGNAL,
               pdTRUE, pdFALSE, osWaitForever);

    StatusLED_AllOutputs();

    if (uxBits & RED_LED_SIGNAL)
    {
      for (i = 0; i < 6; i++)
      {
        HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
        osDelay(60);
        HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
        osDelay(60);
      }
      StatusLED_BlinkRedLED();
    }
    else if (uxBits & GRN_LED_SIGNAL)
    {
      for (i = 0; i < 6; i++)
      {
        HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_RESET);
        osDelay(60);
        HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_SET);
        osDelay(60);
      }
      StatusLED_BlinkGreenLED();
    }
    else if (uxBits & BLU_LED_SIGNAL)
    {
      /* Solid blue ~0.5s, then rapid blink, then LPTIM slow blink */
      HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_RESET);
      osDelay(125);
      HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_SET);
      osDelay(5);
      for (i = 0; i < 5; i++)
      {
        HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_RESET);
        osDelay(5);
        HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_SET);
        osDelay(5);
      }
      StatusLED_BlinkBlueLED();
    }
    else if (uxBits & REDBLUGRN_LED_SIGNAL)
    {
      for (i = 0; i < 6; i++)
      {
        HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_RESET);
        osDelay(60);
        HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_SET);
        osDelay(60);
      }
      StatusLED_BlinkRedBlueGreenLED();
    }
    else /* GRNBLU or hall effect */
    {
      for (i = 0; i < 6; i++)
      {
        HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_RESET);
        osDelay(60);
        HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_SET);
        osDelay(60);
      }
      StatusLED_BlinkBlueGreenLED();
    }
  }
}

/* ================================================================== */
/*                     NEW TASKS AND HELPERS                          */
/* ================================================================== */

static void InitializeCodecAndDMA(void)
{
  g_stop2Allowed = 0;

  __HAL_RCC_I2C1_CLK_ENABLE();
  AudioCodec_Initialize(&hi2c1);
  __HAL_RCC_I2C1_CLK_DISABLE();

  MX_GPDMA1_Init();
  MX_SAI1_Init(WAVFile.SampleRate ? WAVFile.SampleRate : 32000);
  HAL_SAI_RegisterCallback(&hsai_BlockA1, HAL_SAI_RX_COMPLETE_CB_ID, AudioTransferComplete);
  HAL_SAI_RegisterCallback(&hsai_BlockA1, HAL_SAI_RX_HALFCOMPLETE_CB_ID, AudioTransferHalfComplete);
  HAL_SAI_Receive_DMA(&hsai_BlockA1, I2SRxBuffer, BUFFER_SIZE);
}

static void WriteSystemLog(const char *inputString)
{
  char stringBuffer[200];
  char stringDate[50];

  RTC_ReturnDateTimeStampString(stringDate);
  snprintf(stringBuffer, sizeof(stringBuffer), "%s %s\r\n", stringDate, inputString);
  WriteFlashNextEntry(stringBuffer);
}

static void WriteDebugFile(void)
{
  static FIL DebugFile;
  UINT bw;
  char StringBuffer[100];
  uint8_t StringLength;

  f_chdir("0:/");
  if (f_open(&DebugFile, "DebugLog.txt", FA_OPEN_APPEND | FA_WRITE) != FR_OK)
    return;

  if (DebugFile.obj.objsize > 60000)
  {
    f_lseek(&DebugFile, 0);
    f_truncate(&DebugFile);
  }

  f_write(&DebugFile, "*************************************\r\n** SwiftOSH boot sequence occurred **\r\n", 78, &bw);
  RTC_ReturnDateTimeString(StringBuffer);
  f_write(&DebugFile, StringBuffer, 32, &bw);

  uint32_t SerialNumber = *(__IO uint32_t *)(0x0BFA0700);
  StringLength = snprintf(StringBuffer, sizeof(StringBuffer), "Serial number: %lu", (unsigned long)SerialNumber);
  f_write(&DebugFile, StringBuffer, StringLength, &bw);

  StringLength = snprintf(StringBuffer, sizeof(StringBuffer), "\r\nFirmware version: %d.%d.%d.%d\r\n",
    __SWIFT_FIRMWARE_VERSION_MAIN, __SWIFT_FIRMWARE_VERSION_SUB1,
    __SWIFT_FIRMWARE_VERSION_SUB2, __SWIFT_FIRMWARE_VERSION_RC);
  f_write(&DebugFile, StringBuffer, StringLength, &bw);

  float BattVoltage = GetBatteryVoltage();
  StringLength = snprintf(StringBuffer, sizeof(StringBuffer), "Battery voltage: %2.2f V\r\n", (double)BattVoltage);
  f_write(&DebugFile, StringBuffer, StringLength, &bw);

  StringLength = AudioFiles_GetDiskInfoString(StringBuffer);
  f_write(&DebugFile, StringBuffer, StringLength, &bw);

  f_close(&DebugFile);
}

static void WriteSystemFlashLogToSD(void)
{
  static FIL SystemFile;
  UINT bw;
  char StringBuffer[200];
  uint8_t StringLength;
  char read_buffer[MAX_STRING_LENGTH];

  if (f_open(&SystemFile, "0:/System.log", FA_OPEN_APPEND | FA_WRITE) != FR_OK)
    return;

  if (SystemFile.obj.objsize > 100000)
  {
    f_lseek(&SystemFile, 0);
    f_truncate(&SystemFile);
  }

  uint32_t current_addr = FLASH_USER_START_ADDR;
  while (current_addr < FLASH_USER_END_ADDR)
  {
    if (Flash_IsEmpty(current_addr, 16))
      break;

    uint32_t string_length = *((uint32_t *)current_addr);
    if (string_length == 0 || string_length > MAX_STRING_LENGTH)
      break;

    if (Flash_ReadString(current_addr, read_buffer, sizeof(read_buffer)) == FLASH_OK)
    {
      StringLength = snprintf(StringBuffer, sizeof(StringBuffer), "%s", read_buffer);
      f_write(&SystemFile, StringBuffer, StringLength, &bw);
    }

    uint32_t entry_size = 16 + ((string_length + 15) & ~15);
    current_addr += entry_size;
  }

  Flash_ErasePage(FLASH_USER_START_ADDR, FLASH_USER_END_ADDR);
  f_close(&SystemFile);
}

/**
  * @brief  InitializeScheduleTask — full port from SwiftOne.
  */
void InitializeScheduleTask(void const *argument)
{
  (void)argument;
  WriteSystemLog("** ISchedule Task Running **");
  EventBits_t uxBits;

  for (;;)
  {
    uint8_t InitialScheduleState = RTC_InitializeScheduleAlarms(&SwiftSchedule);

    switch (InitialScheduleState)
    {
    case 1:
      WriteSystemLog("<ISchedule> Middle of recording period");
      g_stop2Allowed = 0;
      StatusLED_SolidGreenLED();
      __HAL_RCC_SDMMC1_CLK_ENABLE();
      AudioFiles_NewRecordingDirectory(&WAVFile);
      AudioFiles_NewDayDirectory(WAVFile.Filename);
      AudioFiles_NewAudioFile(WAVFile.Filename, DSTSettings.DST_Suffix_To_Use, &FsFile);
      WAVFile.DirectoryLevel = 2;
      BufferWritesPerFile = WAVFile.FileSize;
      InitializeCodecAndDMA();
      xEventGroupSetBits(xEventGroup, GRN_LED_SIGNAL);
      osThreadResume(RecordLoopHandle);
      osThreadSuspend(NULL);
      break;

    case 2:
      WriteSystemLog("<ISchedule> Between recording periods");
      g_stop2Allowed = 1;
      xEventGroupSetBits(xEventGroup, BLU_LED_SIGNAL);
      uxBits = xEventGroupWaitBits(xEventGroup,
                 BUTTON_SIGNAL | ALARM_A_SIGNAL | NEW_VOICE_MEMO_SIGNAL |
                 LOW_BATTERY_SIGNAL | END_OF_DAY_SIGNAL,
                 pdTRUE, pdFALSE, osWaitForever);
      g_stop2Allowed = 0;
      __HAL_RCC_SDMMC1_CLK_ENABLE();

      if (uxBits & END_OF_DAY_SIGNAL)
      {
        uint8_t returnFlag = RTC_CheckIfBetweenDates(SwiftSchedule.StartDate, SwiftSchedule.StopDate);
        if (returnFlag == 1)
        {
          AudioFiles_NewRecordingDirectory(&WAVFile);
          AudioFiles_NewDayDirectory(WAVFile.Filename);
          WAVFile.DirectoryLevel = 2;
          if (SwiftSchedule.StartsAtMidnightFlag == 1)
          {
            StatusLED_SolidGreenLED();
            AudioFiles_NewAudioFile(WAVFile.Filename, DSTSettings.DST_Suffix_To_Use, &FsFile);
            SwiftSchedule.CurrentRecordNumber = 0;
            SetNextStopTime(&SwiftSchedule);
            InitializeCodecAndDMA();
            xEventGroupSetBits(xEventGroup, GRN_LED_SIGNAL);
            osThreadResume(RecordLoopHandle);
            osThreadSuspend(NULL);
          }
          else
          {
            SwiftSchedule.CurrentRecordNumber = SwiftSchedule.NumberOfRecords;
            SetNextStartTime(&SwiftSchedule);
            osThreadResume(StandbyHandle);
            osThreadSuspend(NULL);
          }
        }
        else if (returnFlag == 2)
        {
          SwiftSchedule.CurrentRecordNumber = 0;
          SetNextStartTime(&SwiftSchedule);
          RTC_DeactivateAlarm(RTC_ALARM_A);
          RTC_DeactivateAlarm(RTC_ALARM_B);
        }
      }
      else if (uxBits & LOW_BATTERY_SIGNAL)
      {
        osThreadResume(LowBatteryHandle);
        xEventGroupSetBits(xEventGroup, LOW_BATTERY_SIGNAL);
        osThreadTerminate(NULL);
        break;
      }
      else if (uxBits & (BUTTON_SIGNAL | ALARM_A_SIGNAL))
      {
        StatusLED_SolidGreenLED();
        AudioFiles_NewRecordingDirectory(&WAVFile);
        AudioFiles_NewDayDirectory(WAVFile.Filename);
        AudioFiles_NewAudioFile(WAVFile.Filename, DSTSettings.DST_Suffix_To_Use, &FsFile);
        BufferWritesPerFile = WAVFile.FileSize;
        if (uxBits & BUTTON_SIGNAL)
        {
          SwiftSchedule.EndsAtMidnightFlag = 0;
          SwiftSchedule.StartsAtMidnightFlag = 1;
          SwiftSchedule.CurrentRecordNumber = 0;
          SwiftSchedule.NumberOfRecords = 1;
          SwiftSchedule.ScheduleType = SCHEDULE_TYPE_CONTINUOUS;
          SwiftSchedule.StartDate.Year = 0; SwiftSchedule.StartDate.Date = 1; SwiftSchedule.StartDate.Month = 1;
          SwiftSchedule.StopDate.Year = 99; SwiftSchedule.StopDate.Date = 1; SwiftSchedule.StopDate.Month = 1;
          SwiftSchedule.StartStopTimes[0].RecordingStartTime.Hours = 0x00;
          SwiftSchedule.StartStopTimes[0].RecordingStartTime.Minutes = 0x00;
          SwiftSchedule.StartStopTimes[0].RecordingStopTime.Hours = 0x23;
          SwiftSchedule.StartStopTimes[0].RecordingStopTime.Minutes = 0x59;
          SetNextStopTime(&SwiftSchedule);
          RTC_EnableAlarmB(SET_ALARMB_MIDNIGHT);
        }
        else
        {
          SetNextStopTime(&SwiftSchedule);
        }
        InitializeCodecAndDMA();
        xEventGroupSetBits(xEventGroup, GRN_LED_SIGNAL);
        osThreadResume(RecordLoopHandle);
      }
      else if (uxBits & NEW_VOICE_MEMO_SIGNAL)
      {
        osThreadDef(VoiceMemo, NewVoiceMemoTask, osPriorityNormal, 0, 512);
        VoiceMemoHandle = osThreadCreate(osThread(VoiceMemo), NULL);
      }
      osThreadSuspend(NULL);
      break;

    default:
      xEventGroupSetBits(xEventGroup, BLU_LED_SIGNAL);
      __HAL_RCC_I2C1_CLK_ENABLE();
      AudioCodec_SleepMode(&hi2c1);
      __HAL_RCC_I2C1_CLK_DISABLE();
      osThreadResume(StandbyHandle);
      osThreadSuspend(NULL);
      break;
    }
  }
}

/**
  * @brief  Voice memo task — records a short memo triggered by hall effect switch.
  */
void NewVoiceMemoTask(void const *argument)
{
  (void)argument;
  FRESULT ReturnResult;
  uint8_t Filename[6] = {77, 69, 77, 79, 95, 0};
  uint32_t LoopFlag = 1;
  EventBits_t uxBits;
  UINT byteswritten;
  FRESULT FatFsResult;

  for (;;)
  {
    xEventGroupSetBits(xEventGroup, GRNBLU_LED_SIGNAL);
    f_chdir("0:/");
    ReturnResult = f_stat("VoiceMemos", NULL);
    if (ReturnResult == FR_NO_FILE)
    {
      if (f_mkdir("VoiceMemos") != FR_OK)
      {
        xEventGroupSetBits(xEventGroup, ERROR_SIGNAL);
        osThreadTerminate(NULL);
      }
    }
    if (f_chdir("VoiceMemos") != FR_OK)
    {
      xEventGroupSetBits(xEventGroup, ERROR_SIGNAL);
      osThreadTerminate(NULL);
    }
    if (AudioFiles_NewAudioFile(Filename, DSTSettings.DST_Suffix_To_Use, &VoiceMemoFile))
    {
      xEventGroupSetBits(xEventGroup, ERROR_SIGNAL);
      osThreadTerminate(NULL);
    }

    __HAL_RCC_I2C1_CLK_ENABLE();
    AudioCodec_Initialize(&hi2c1);
    __HAL_RCC_I2C1_CLK_DISABLE();
    RTC_AddTimeAndSetAlarmA();

    MX_GPDMA1_Init();
    MX_SAI1_Init(WAVFile.SampleRate ? WAVFile.SampleRate : 32000);
    HAL_SAI_RegisterCallback(&hsai_BlockA1, HAL_SAI_RX_COMPLETE_CB_ID, AudioTransferComplete);
    HAL_SAI_RegisterCallback(&hsai_BlockA1, HAL_SAI_RX_HALFCOMPLETE_CB_ID, AudioTransferHalfComplete);
    HAL_SAI_Receive_DMA(&hsai_BlockA1, I2SRxBuffer, BUFFER_SIZE);

    while (LoopFlag)
    {
      uxBits = xEventGroupWaitBits(xEventGroup,
                 WRITEBUFFER1_SIGNAL | WRITEBUFFER2_SIGNAL |
                 ALARM_A_SIGNAL | BUTTON_SIGNAL | NEW_VOICE_MEMO_SIGNAL,
                 pdTRUE, pdFALSE, osWaitForever);
      if (uxBits & WRITEBUFFER1_SIGNAL)
      {
        FatFsResult = f_write(&VoiceMemoFile, &I2SRxBuffer[0], (UINT)(BUFFER_SIZE_DIV2), (void *)&byteswritten);
        if (FatFsResult != FR_OK) { xEventGroupSetBits(xEventGroup, ERROR_SIGNAL); osThreadTerminate(NULL); }
      }
      else if (uxBits & WRITEBUFFER2_SIGNAL)
      {
        FatFsResult = f_write(&VoiceMemoFile, &I2SRxBuffer[BUFFER_SIZE_DIV2], (UINT)(BUFFER_SIZE_DIV2), (void *)&byteswritten);
        if (FatFsResult != FR_OK) { xEventGroupSetBits(xEventGroup, ERROR_SIGNAL); osThreadTerminate(NULL); }
      }
      else if (uxBits & (BUTTON_SIGNAL | NEW_VOICE_MEMO_SIGNAL | ALARM_A_SIGNAL))
      {
        HAL_SAI_DMAStop(&hsai_BlockA1);
        __HAL_SAI_DISABLE(&hsai_BlockA1);
        HAL_SAI_MspDeInit(&hsai_BlockA1);
        hsai_BlockA1.State = HAL_SAI_STATE_RESET;
        LoopFlag = 0;
        AudioFiles_WriteHeader(&SwiftErrors, &VoiceMemoFile);
      }
    }

    LoopFlag = 1;
    AudioFiles_CloseFile(&SwiftErrors, &VoiceMemoFile);
    RTC_DeactivateAlarm(RTC_ALARM_A);
    osThreadResume(InitializeScheduleHandle);
    osThreadTerminate(VoiceMemoHandle);
  }
}

/**
  * @brief  Low battery task — blinks red-red-blue pattern indefinitely.
  */
void LowBatteryTask(void const *argument)
{
  (void)argument;
  for (;;)
  {
    xEventGroupWaitBits(xEventGroup, LOW_BATTERY_SIGNAL, pdTRUE, pdTRUE, osWaitForever);
    StatusLED_LowBatteryMode();
    while (1)
    {
      HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
      osDelay(60);
      HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
      osDelay(60);
      HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
      osDelay(60);
      HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
      osDelay(60);
      HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_RESET);
      osDelay(60);
      HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_SET);
      osDelay(60);
      osDelay(5000);
    }
  }
}

/* ================================================================== */
/*                     HAL TIMEBASE CALLBACK                          */
/* ================================================================== */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
    HAL_IncTick();
}

/* ================================================================== */
/*                     ERROR HANDLER                                  */
/* ================================================================== */

void Error_Handler(void)
{
  __disable_irq();

  /* Safe fallback: manually blink RED LED (PC1) using GPIO registers directly.
     This works even if no clocks or peripherals have been initialized yet,
     because GPIOC clock may already be on from VBUS check or GPIO init.
     If not, we enable it here. */
  RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOCEN;
  /* Small delay for clock to stabilize */
  volatile uint32_t dummy = RCC->AHB2ENR1;
  (void)dummy;

  /* PC1 = output push-pull */
  GPIOC->MODER   = (GPIOC->MODER & ~(3UL << (1 * 2))) | (1UL << (1 * 2));
  GPIOC->OTYPER &= ~(1UL << 1);
  GPIOC->OSPEEDR &= ~(3UL << (1 * 2));

  /* Blink RED LED forever — visible proof we hit Error_Handler
     RED (PC1) is active-high: HIGH = on, LOW = off */
  while (1)
  {
    GPIOC->BSRR = (1UL << 1);       /* PC1 high = RED on */
    for (volatile uint32_t i = 0; i < 200000; i++) {}
    GPIOC->BSRR = (1UL << (1 + 16)); /* PC1 low = RED off */
    for (volatile uint32_t i = 0; i < 200000; i++) {}
  }
}

/* ================================================================== */
/*                     HAL TICK OVERRIDE                              */
/* ================================================================== */

/**
  * @brief  Override HAL_InitTick to prevent SysTick configuration.
  *         FreeRTOSConfig.h maps SysTick_Handler to xPortSysTickHandler,
  *         so any SysTick interrupt before the scheduler starts will crash.
  *         TIM6 is used as the HAL timebase once initialized later.
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  (void)TickPriority;
  return HAL_OK;
}

/* ================================================================== */
/*                           MAIN                                     */
/* ================================================================== */

int main(void)
{
  HAL_Init();

  /* Check VBUS to decide USB mode vs recording mode */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin  = VBUS_SCALED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_SCALED_GPIO_Port, &GPIO_InitStruct);

  if (HAL_GPIO_ReadPin(VBUS_SCALED_GPIO_Port, VBUS_SCALED_Pin) == GPIO_PIN_SET)
  {
    /* ---- USB MODE ---- */
    SystemClock_Config_USB();
    MX_RTC_Init();
    MX_TIM6_Init();       /* HAL timebase - USB stack uses HAL_Delay */
    MX_GPIO_Init_USB();
    MX_ADC1_Init();
    MX_USB_DEVICE_Init();
  }
  else
  {
    /* ---- RECORDING MODE ---- */
    SystemClock_Config();
    MX_RTC_Init();
    MX_TIM6_Init();       /* HAL timebase so HAL_Delay works */
    MX_GPIO_Init();
    MX_SDMMC1_SD_Init();
    MX_LPTIM1_Init();

    StatusLED_Initialize();

    HAL_Delay(1000);  /* Allow voltage to settle for accurate battery read */

    xEventGroup = xEventGroupCreate();

    osThreadDef(InitializeFATFS, InitializeFATTask, osPriorityIdle, 0, 768);
    InitializeFATFSHandle = osThreadCreate(osThread(InitializeFATFS), NULL);

    osKernelStart();
  }

  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
