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
#include "SDCardConfig.h"
#include "SDCardSchedule.h"
#include "SD_FW_Update.h"
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
osThreadId ButtonHandle;

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
#define SD_INSERTED_SIGNAL    ( 1 << 17 )
#define BUTTON_PRESS_SIGNAL   ( 1 << 18 )  /* raw ISR event — ButtonTask checks hold duration */

/* ---- DMA audio buffer ---- */
/* BUFFER_SIZE is in bytes. DMA element count = BUFFER_SIZE / 2 (halfwords).
   Half-complete fires at BUFFER_SIZE_DIV2 bytes = BUFFER_SIZE/4 halfwords.

   CRITICAL: STM32U5 GPDMA CBR1.BNDT is 16-bit — max block size = 65535 bytes.
   Each DMA node transfers BUFFER_SIZE_DIV2 bytes, so BUFFER_SIZE_DIV2 must be <= 65535.

   Using maximum allowable half-buffer = 65534 bytes (even, for 16-bit halfword alignment):
     full buffer = 131068 bytes
     at 96 kHz mono 16-bit: 65534 / 2 / 96000 = ~341 ms write window
   RAM cost: ~128 KB static. */
#define BUFFER_SIZE       131068
#define BUFFER_SIZE_DIV2  (BUFFER_SIZE / 2)

/* ---- One-off compile-time sample rate override ---- */
/* Uncomment to force a specific sample rate regardless of flash settings.
   Useful for testing. Remove or comment out for normal operation. */
//#define FORCE_SAMPLE_RATE  192000
uint8_t I2SRxBuffer[BUFFER_SIZE] __attribute__((section(".bss")));

/* ---- DMA linked-list objects for circular SAI1 RX ---- */
/* Two nodes needed: GPDMA linked-list mode has no half-transfer interrupt.
   Node1 transfers first half → TC fires (half-complete callback).
   Node2 transfers second half → TC fires (full-complete callback). */
DMA_NodeTypeDef  SAI_DMA_Node1 __attribute__((aligned(32)));
DMA_NodeTypeDef  SAI_DMA_Node2 __attribute__((aligned(32)));
DMA_QListTypeDef SAI_DMA_Queue;

/* ---- Stop-2 gate (same logic as original) ---- */
volatile uint8_t g_initComplete = 0;
volatile uint8_t g_stop2Allowed = 1;

/* ---- Battery voltage cache (updated in USB main loop, read by GET_REPORT handler) ---- */
volatile float g_CachedBatteryVoltage = -1.0f;

/* ---- USB suspend flag — set from PCD callback, checked in main loop ---- */
volatile uint32_t g_usbSuspendTick = 0;

/* ---- Settings / state variables ---- */
WAVFile_Attributes WAVFile;
Swift_Schedule     SwiftSchedule;
SWIFT_ERRORS       SwiftErrors;
DST_Settings       DSTSettings;
unsigned int       BufferWritesPerFile;

/* ---- Day-skip recording interval ---- */
static uint8_t  g_daySkip = 0;        /* 0=disabled, N=record every Nth day */
static uint8_t  g_daySkipCounter = 0; /* counts midnights since last recording day */

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
static void MX_SAI1_ReconfigPLL2(uint32_t AudioSampleRate);
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
void ButtonTask(void const *argument);

static void InitializeCodecAndDMA(void);
static void WriteSystemLog(const char *inputString);
static void WriteSystemFlashLogToSD(void);
static void CheckAndDumpFlashLogs(void);
static uint8_t ShouldSkipToday(void);
static void WriteDebugFile(void);

/* Set from ISR, logged from task context to avoid flash writes in ISR */
static volatile uint32_t g_saiErrorCode;
static volatile uint32_t g_dmaErrorCode;

float GetBatteryVoltage(void);
void  USB_HID_ProcessFlash(void);

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

  /* Peripheral clocks: RTC=LSE, LPTIM1=LSE, LPTIM2=LSE, I2C1=PCLK1,
     SDMMC1=PLL1-P, ADC=SYSCLK.
     SAI1 clock is NOT configured here — MX_SAI1_ReconfigPLL2() sets up PLL2
     and switches SAI1 to it immediately before each recording session. */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC   | RCC_PERIPHCLK_LPTIM1 |
                                       RCC_PERIPHCLK_LPTIM2 | RCC_PERIPHCLK_I2C1  |
                                       RCC_PERIPHCLK_SDMMC |
                                       RCC_PERIPHCLK_ADCDAC;
  PeriphClkInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_LSE;
  PeriphClkInit.Lptim2ClockSelection = RCC_LPTIM2CLKSOURCE_LSE;
  PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.SdmmcClockSelection  = RCC_SDMMCCLKSOURCE_PLL1;
  PeriphClkInit.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_SYSCLK;
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
  hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV4;  /* 160MHz/4 = 40MHz, within 50MHz max */
  hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait      = DISABLE;
  hadc1.Init.ContinuousConvMode    = DISABLE;
  hadc1.Init.NbrOfConversion       = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
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

  /* Start in one-shot mode with interrupt — button falling edge (ETR/TRIGSOURCE_0)
     triggers a single count-up to Period; AutoReloadMatchCallback fires if button
     is still held. On STM32U5 HAL, SetOnce_Start_IT takes (hlptim, Channel). */
  if (HAL_LPTIM_SetOnce_Start_IT(&hlptim1, LPTIM_CHANNEL_1) != HAL_OK)
    Error_Handler();
}

static void MX_SAI1_Init(uint32_t AudioSampleRate)
{
  /* SAI1 Block A — master receiver, standard I2S slave protocol
     PA8=BCLK, PA9=WCLK(FSYNC), PA10=DOUT(SD_A), PB8=MCLK
     
     TLV320ADC3120 I2S timing (datasheet Fig 8-5):
       - FSYNC falls → 1 BCLK idle → MSB of left channel (slot 0)
       - FSYNC rises → 1 BCLK idle → MSB of right channel (slot 1)
       - Data transmitted on falling BCLK edge → SAI samples on rising edge
     
     SAI free-protocol settings for standard I2S:
       - FSOffset = SAI_FS_BEFOREFIRSTBIT: FSYNC goes active 1 BCLK before data
       - FSDefinition = SAI_FS_CHANNEL_IDENTIFICATION: FS low=left, FS high=right
       - FSPolarity = SAI_FS_ACTIVE_LOW: left channel when FSYNC is low (standard I2S)
       - ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE: sample on rising BCLK (codec outputs on falling)
       - FrameLength = 64: 2 slots × 32 bits = 64 BCLK per frame
       - ActiveFrameLength = 32: FSYNC low for 32 BCLK (left half-frame)
       - SlotActive = both slots: receive left (slot 0) and right (slot 1)
       - FirstBitOffset = 0: data starts immediately after the 1-BCLK FSYNC offset
  */
  hsai_BlockA1.Instance            = SAI1_Block_A;
  hsai_BlockA1.Init.Protocol       = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode      = SAI_MODEMASTER_RX;
  hsai_BlockA1.Init.DataSize       = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit       = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing  = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockA1.Init.Synchro        = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive    = SAI_OUTPUTDRIVE_ENABLE;
  hsai_BlockA1.Init.NoDivider      = SAI_MASTERDIVIDER_ENABLE;
  /* MckOverSampling=ENABLE halves the effective MCKDIV (OSR=1: MCKDIV = SAI_clk / (Fs×512)).
     For low rates (≤24kHz) this would require MCKDIV > 63 (the 6-bit field max).
     Use DISABLE for ≤24kHz: MCKDIV = SAI_clk / (Fs×512) without the ×2 penalty.
       OSR=DISABLE: 8kHz→24, 16kHz→12, 24kHz→8  (all ≤63 ✓)
       OSR=ENABLE:  32kHz→6, 48kHz→4, 96kHz→2, 192kHz→1 (all ≤63 ✓) */
  hsai_BlockA1.Init.MckOverSampling = (AudioSampleRate <= 24000) ? SAI_MCK_OVERSAMPLING_DISABLE
                                                                  : SAI_MCK_OVERSAMPLING_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold  = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = AudioSampleRate;
  hsai_BlockA1.Init.SynchroExt     = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_MONOMODE;  /* Only left slot (slot 0) written to DMA buffer */
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.PdmInit.Activation  = DISABLE;
  hsai_BlockA1.Init.PdmInit.MicPairsNbr = 0;
  hsai_BlockA1.Init.PdmInit.ClockEnable = SAI_PDM_CLOCK1_ENABLE;

  /* Frame: 64 BCLK total, FSYNC low for 32 BCLK (left half), standard I2S offset */
  hsai_BlockA1.FrameInit.FrameLength       = 64;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 32;
  hsai_BlockA1.FrameInit.FSDefinition      = SAI_FS_CHANNEL_IDENTIFICATION;
  hsai_BlockA1.FrameInit.FSPolarity        = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset          = SAI_FS_BEFOREFIRSTBIT;

  /* Slots: 2 × 32-bit in frame, but only slot 0 (left/ch1) active for mono.
     Codec outputs mono on channel 1 (slot 0 in I2S). Slot 1 is ignored. */
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize       = SAI_SLOTSIZE_32B;
  hsai_BlockA1.SlotInit.SlotNumber     = 2;
  hsai_BlockA1.SlotInit.SlotActive     = SAI_SLOTACTIVE_0;  /* Mono: only left channel */

  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
    Error_Handler();
}

/**
  * @brief  Reconfigure PLL2 for the requested audio sample rate before SAI init.
  *
  *  HSE = 12.288 MHz. One PLL2 config covers all supported rates:
  *    PLL2M=1, PLL2N=32, PLL2P=4 → VCO=393.216 MHz, PLL2P=98.304 MHz
  *
  *  MCKDIV = SAI_clk / (Fs × 512) [OSR=DISABLE] or / (Fs × 1024) [OSR=ENABLE]:
  *    OSR=DISABLE (<=24kHz):  8kHz->24, 16kHz->12, 24kHz->8   (all within 6-bit max of 63)
  *    OSR=ENABLE  (>=32kHz): 32kHz->6, 48kHz->4,  96kHz->2, 192kHz->1
  *
  *  Any unsupported rate falls back to 32kHz.
  */
static void MX_SAI1_ReconfigPLL2(uint32_t AudioSampleRate)
{
  RCC_PeriphCLKInitTypeDef clk = {0};

  /* Step 1: Switch SAI1 clock source away from PLL2 so PLL2 can be stopped.
     HAL_RCCEx_PeriphCLKConfig cannot reconfigure a PLL while it is the active
     clock source for a peripheral — it hangs waiting for PLL2RDY to clear.
     HSI16 is always available as a safe temporary source. */
  clk.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  clk.Sai1ClockSelection   = RCC_SAI1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK)
    Error_Handler();

  /* Step 2: Disable PLL2 and wait for it to stop.
     HAL V1.7.0 has no PLL2State field — use the direct register macro. */
  __HAL_RCC_PLL2_DISABLE();
  uint32_t tickstart = HAL_GetTick();
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_PLL2RDY) != 0U)
  {
    if ((HAL_GetTick() - tickstart) > 2U)
      break;  /* should stop in <1ms; don't hang if something is wrong */
  }

  /* Step 3: Reconfigure PLL2 and switch SAI1 back to it.
     HAL_RCCEx_PeriphCLKConfig handles enabling PLL2 and waiting for lock.
     HSE = 12.288 MHz. PLL2M=1, PLL2N=32, PLL2P=4 → 98.304 MHz.
     BCLK = 64 × Fs. MCKDIV = 98304000 / (Fs × 64):
       8kHz→192, 16kHz→96, 24kHz→64, 32kHz→48, 48kHz→32, 96kHz→16, 192kHz→8 */
  clk.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
  clk.Sai1ClockSelection   = RCC_SAI1CLKSOURCE_PLL2;
  clk.PLL2.PLL2Source      = RCC_PLLSOURCE_HSE;
  clk.PLL2.PLL2M           = 1;   /* 12.288 MHz / 1 = 12.288 MHz ref */
  clk.PLL2.PLL2N           = 32;  /* × 32 = 393.216 MHz VCO */
  clk.PLL2.PLL2P           = 4;   /* / 4  = 98.304 MHz → SAI1 */
  clk.PLL2.PLL2Q           = 4;
  clk.PLL2.PLL2R           = 4;
  clk.PLL2.PLL2RGE         = RCC_PLLVCIRANGE_1;  /* 8–16 MHz input range */
  clk.PLL2.PLL2FRACN       = 0;
  clk.PLL2.PLL2ClockOut    = RCC_PLL2_DIVP;
  if (HAL_RCCEx_PeriphCLKConfig(&clk) != HAL_OK)
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

  /* VBAT_MONITOR_EN (PB5) — enable battery voltage divider */
  HAL_GPIO_WritePin(VBAT_MONITOR_GPIO_Port, VBAT_MONITOR_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin  = VBAT_MONITOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(VBAT_MONITOR_GPIO_Port, &GPIO_InitStruct);

  /* PS/SYNC (PC2) — low for power-save mode */
  HAL_GPIO_WritePin(PS_SYNC_GPIO_Port, PS_SYNC_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin  = PS_SYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PS_SYNC_GPIO_Port, &GPIO_InitStruct);

  /* Pushbutton (PC6) — falling-edge EXTI, internal pull-up */
  GPIO_InitStruct.Pin  = PUSHBUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PUSHBUTTON_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI6_IRQn);

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

  /* VBUS_SCALED (PA15) — USB detect, rising-edge EXTI triggers reset into USB mode */
  GPIO_InitStruct.Pin  = VBUS_SCALED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(VBUS_SCALED_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI15_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* Blue LED solid on when USB connected */
  GPIO_InitStruct.Pin   = BLU_LED_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLU_LED_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_SET);

  /* VBAT_SCALED (PA0) — analog input for ADC */
  GPIO_InitStruct.Pin  = VBAT_SCALED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBAT_SCALED_GPIO_Port, &GPIO_InitStruct);

  /* VBUS_SCALED (PA15) — explicitly init as input with pull-down.
     When USB is unplugged, VBUS back-feeds ~2.45V through the USB peripheral's
     internal VDDUSB regulator, holding VBUS_SCALED at ~1.75V. The STM32U5
     internal pull-down (~40k) combined with R21 (82k) creates a divider that
     pulls PA15 below the GPIO logic threshold when VBUS is truly gone. */
  GPIO_InitStruct.Pin  = VBUS_SCALED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(VBUS_SCALED_GPIO_Port, &GPIO_InitStruct);

  /* ADC_EN (PB15) and VBAT_MONITOR_EN (PB5) — output, default LOW */
  HAL_GPIO_WritePin(ADC_EN_GPIO_Port, ADC_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(VBAT_MONITOR_GPIO_Port, VBAT_MONITOR_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin  = ADC_EN_Pin | VBAT_MONITOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* ================================================================== */
/*                     BATTERY VOLTAGE                                */
/* ================================================================== */

float GetBatteryVoltage(void)
{
  float battery_v = -1.0f;

  /* Enable battery voltage divider, allow settling */
  HAL_GPIO_WritePin(VBAT_MONITOR_GPIO_Port, VBAT_MONITOR_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  /* ADC is initialized once at startup — just start/poll/read/stop */
  HAL_ADC_Start(&hadc1);
  HAL_StatusTypeDef poll_status  = HAL_ADC_PollForConversion(&hadc1, 100);

  uint16_t raw = 0;
  if (poll_status == HAL_OK)
  {
    raw = HAL_ADC_GetValue(&hadc1);
    /* Divider ratio 10/(10+33) = 0.2326, so Vin = (raw/4095)*3.3/0.2326 */
    battery_v = (raw * 3.3f / 4095.0f) * 4.298f;
  }

  HAL_ADC_Stop(&hadc1);

  /* Disable voltage divider to save power */
  HAL_GPIO_WritePin(VBAT_MONITOR_GPIO_Port, VBAT_MONITOR_Pin, GPIO_PIN_RESET);

  return battery_v;
}


/* ================================================================== */
/*                     SAI DMA CALLBACKS                              */
/* ================================================================== */

/* GPDMA linked-list mode has no half-transfer interrupt. We use two nodes:
   Node1 → first half, Node2 → second half. Both generate TC events.
   This custom DMA callback checks which node just completed and signals
   the appropriate FreeRTOS event. Toggle flag tracks position in cycle. */
static volatile uint8_t g_dmaHalfToggle = 0;
/* Skip the first two DMA signals after each recording start — the buffer fills
   with silence while the codec PLL locks. Two signals = one full buffer. */
static volatile uint8_t g_dmaWarmupSkip = 0;

static void DMA_SAI_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
  (void)hdma;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (g_dmaHalfToggle == 0)
  {
    /* Node1 just completed → first half of buffer is ready */
    xEventGroupSetBitsFromISR(xEventGroup, WRITEBUFFER1_SIGNAL, &xHigherPriorityTaskWoken);
    g_dmaHalfToggle = 1;
  }
  else
  {
    /* Node2 just completed → second half of buffer is ready */
    xEventGroupSetBitsFromISR(xEventGroup, WRITEBUFFER2_SIGNAL, &xHigherPriorityTaskWoken);
    g_dmaHalfToggle = 0;
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* NOTE: These SAI callbacks are unused — GPDMA linked-list mode uses
   DMA_SAI_XferCpltCallback instead. Kept for reference/fallback.
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
*/

/* SAI error callback — captures error codes from ISR, logged from task context.
   Registered via HAL_SAI_RegisterCallback if SAI error detection is needed. */
__attribute__((unused))
static void SAI_ErrorDiag(SAI_HandleTypeDef *hsai)
{
  g_saiErrorCode = hsai->ErrorCode;
  g_dmaErrorCode = hsai->hdmarx ? hsai->hdmarx->ErrorCode : 0xDEAD;
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
  /* VBUS rising = USB cable plugged in during recording/standby — reset into USB mode */
  else if (GPIO_Pin == VBUS_SCALED_Pin)
  {
    NVIC_SystemReset();
  }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  /* PC6 falling = button pressed — ButtonTask will confirm 1.5s hold */
  if (GPIO_Pin == PUSHBUTTON_Pin)
  {
    if (xEventGroup != NULL)
    {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xEventGroupSetBitsFromISR(xEventGroup, BUTTON_PRESS_SIGNAL, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
  /* PC13 falling = SD card inserted (active-low detect, external pull-up) */
  else if (GPIO_Pin == SDCARD_DETECT_Pin)
  {
    if (xEventGroup != NULL)
    {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xEventGroupSetBitsFromISR(xEventGroup, SD_INSERTED_SIGNAL, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
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

  /* NOTE: Battery voltage check removed from ISR — GetBatteryVoltage() uses
     HAL_Delay and blocking ADC, both illegal in ISR context. Low battery is
     checked in InitializeFATTask before recording starts. */

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
    /* LPTIM1 is used for Stop-2 wakeup only — no action needed here.
       Do NOT re-arm: one-shot mode is triggered by the RTC/LPTIM wakeup path,
       not by this callback. Unconditional re-arm caused a 1.75s periodic ISR
       that corrupted audio DMA timing at 48/96 kHz. */
  }
  else if (hlptim->Instance == LPTIM2)
  {
    /* Auto-reload fires at start of each blink period — turn LEDs ON */
    extern volatile uint8_t led_blink_red, led_blink_grn, led_blink_blu;
    if (led_blink_red)
      HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
    if (led_blink_grn)
      HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_SET);
    if (led_blink_blu)
      HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_SET);
  }
}

void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim)
{
  if (hlptim->Instance == LPTIM2)
  {
    /* Compare fires after BLINK_ON_TICKS (~150ms) — turn LEDs OFF */
    extern volatile uint8_t led_blink_red, led_blink_grn, led_blink_blu;
    if (led_blink_red)
      HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
    if (led_blink_grn)
      HAL_GPIO_WritePin(GRN_LED_GPIO_Port, GRN_LED_Pin, GPIO_PIN_RESET);
    if (led_blink_blu)
      HAL_GPIO_WritePin(BLU_LED_GPIO_Port, BLU_LED_Pin, GPIO_PIN_RESET);
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

  /* Create button hold task */
  osThreadDef(Button, ButtonTask, osPriorityAboveNormal, 0, 128);
  ButtonHandle = osThreadCreate(osThread(Button), NULL);

  /* Create error task (suspended) */
  osThreadDef(Error, ErrorTask, osPriorityBelowNormal, 0, 128);
  ErrorHandle = osThreadCreate(osThread(Error), NULL);
  osThreadSuspend(ErrorHandle);

  /* Create low battery task (suspended) */
  osThreadDef(LowBattery, LowBatteryTask, osPriorityBelowNormal, 0, 256);
  LowBatteryHandle = osThreadCreate(osThread(LowBattery), NULL);
  osThreadSuspend(LowBatteryHandle);

  /* Init SDMMC and mount FatFS.
     Use RTC_BKP_DR2 as a persistent retry counter across reboots.
     Each failed mount increments the counter and delays before rebooting,
     giving the SD card time to settle (slow cards, cold boot, Stop 2 wake).
     After 15 attempts, stop rebooting and blink red permanently. */
  MX_SDMMC1_SD_Init();
  if (AudioFiles_MountSDCard(&SwiftErrors) != 0)
  {
    uint32_t retryCount = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
    retryCount++;
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, retryCount);

    char dbg[48];
    snprintf(dbg, sizeof(dbg), "<FAT Task> SD mount fail, attempt %lu/15", (unsigned long)retryCount);
    WriteSystemLog(dbg);

    if (retryCount >= 15)
    {
      /* Too many failures — stop rebooting, blink red permanently */
      WriteSystemLog("<FAT Task> SD mount failed 15 times, giving up.");
      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, 0);  /* reset for next power cycle */
      osThreadResume(ErrorHandle);
      xEventGroupSetBits(xEventGroup, ERROR_SIGNAL);
      osThreadTerminate(NULL);
    }

    /* Backoff delay: 2s × attempt number (2s, 4s, 6s … up to 30s max) */
    uint32_t delayMs = retryCount * 2000;
    if (delayMs > 30000) delayMs = 30000;
    osDelay(delayMs);

    NVIC_SystemReset();
  }
  WriteSystemLog("<FAT Task> SD Card mounted.");

  /* Check for firmware update .hex file on SD card.
     If found and valid, this does NOT return — MCU resets into new firmware.
     Must be first SD operation after mount. */
  {
    float batt = GetBatteryVoltage();
    FW_Update_CheckAndApply(batt);
  }

  /* Load settings from SD card .cfg file (if present) — must run before
     settings are read from flash.  Renames to .done/.err. */
  SDConfig_CheckAndApply();

  /* Load schedule from SD card .sch file (if present) — must run before
     settings are read from flash.  Renames to .done/.err. */
  SDSchedule_CheckAndApply();

  /* Write debug file */
  WriteDebugFile();

  /* Dump flash logs to SD, then erase flash log area */
  WriteSystemFlashLogToSD();

  /* On-demand flash log dump: if dump_flash_logs.cmd exists on SD, dump
     all flash log entries to a timestamped .out file and erase flash log area.
     WARNING: this disables SDMMC1 clock at the end — all SD access must
     happen before this call in the boot sequence. */
  CheckAndDumpFlashLogs();

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

  /* Init codec over I2C — probe only, puts codec to sleep after */
  MX_I2C1_Init();
  AudioCodec_Initialize(&hi2c1, 0x00, 0x00);  /* gain=0, bias=off for probe */
  AudioCodec_SleepMode(&hi2c1);
  __HAL_RCC_I2C1_CLK_DISABLE();

  /* Read settings from flash */
  SwiftSettings_GetWAVFileAttributes(&WAVFile);
  SwiftSettings_GetSwiftSchedule(&SwiftSchedule);
  SwiftSettings_GetDSTSettings(&DSTSettings);

  /* Read day-skip interval from codec settings byte 14 */
  {
    Codec_Config tmpCodec;
    SwiftSettings_GetCodecVariables(&tmpCodec);
    g_daySkip = tmpCodec.DaySkip;
  }

  /* Initialize DST — when flag is 0 (disabled), use empty suffix so
     filenames don't get garbage appended from uninitialized pointers */
  if (DSTSettings.DST_Active_Flag == 0)
  {
    static uint8_t empty_suffix[] = "";
    DSTSettings.DST_Suffix_To_Use = empty_suffix;
  }
  else
  {
    RTC_InitiateDST(&DSTSettings);
  }

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
    uxBits = xEventGroupWaitBits(xEventGroup,
               WRITEBUFFER1_SIGNAL | WRITEBUFFER2_SIGNAL |
               BUTTON_SIGNAL | ALARM_A_SIGNAL | END_OF_DAY_SIGNAL |
               DST_EVENT_SIGNAL | NEW_VOICE_MEMO_SIGNAL |
               LOW_BATTERY_SIGNAL | ERROR_SIGNAL,
               pdTRUE, pdFALSE, pdMS_TO_TICKS(5000));

    /* Log any deferred SAI/DMA errors captured from ISR context */
    if (g_saiErrorCode || g_dmaErrorCode)
    {
      char dbg[60];
      snprintf(dbg, sizeof(dbg), "<SAI ERR> sai=0x%lX dma=0x%lX\r\n",
               (unsigned long)g_saiErrorCode, (unsigned long)g_dmaErrorCode);
      WriteSystemLog(dbg);
      g_saiErrorCode = 0;
      g_dmaErrorCode = 0;
    }

    /* No DMA signals after 5s timeout — log and keep waiting */
    if (uxBits == 0)
    {
      char dbg[160];
      snprintf(dbg, sizeof(dbg),
               "<RecordLoop> timeout saiState=%d CR1=0x%08lX CCR=0x%08lX CSR=0x%08lX CBR1=0x%08lX SR=0x%08lX MISR=0x%08lX dmaErr=0x%lX\r\n",
               (int)hsai_BlockA1.State,
               (unsigned long)SAI1_Block_A->CR1,
               (unsigned long)GPDMA1_Channel0->CCR,
               (unsigned long)GPDMA1_Channel0->CSR,
               (unsigned long)GPDMA1_Channel0->CBR1,
               (unsigned long)SAI1_Block_A->SR,
               (unsigned long)GPDMA1->MISR,
               (unsigned long)hdma_sai1_a.ErrorCode);
      WriteSystemLog(dbg);
      continue;
    }

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

      AudioFiles_WriteHeader(&SwiftErrors, &FsFile, WAVFile.SampleRate);
      AudioFiles_CloseFile(&SwiftErrors, &FsFile);

      osThreadResume(LowBatteryHandle);
      osThreadSuspend(NULL);
    }
    else if (uxBits & WRITEBUFFER1_SIGNAL)
    {
      if (g_dmaWarmupSkip > 0) { g_dmaWarmupSkip--; continue; }
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
      if (g_dmaWarmupSkip > 0) { g_dmaWarmupSkip--; continue; }
      FatFsResult = f_write(&FsFile, &I2SRxBuffer[BUFFER_SIZE_DIV2], (UINT)(BUFFER_SIZE_DIV2), (void *)&byteswritten);
      if (FatFsResult != FR_OK)
      {
        WriteSystemLog("<RecordLoop> Buffer2 write error");
        xEventGroupSetBits(xEventGroup, ERROR_SIGNAL);
        osThreadTerminate(NULL);
      }

      if (--BufferWritesPerFile == 0)
      {
        WriteSystemLog("<RecordLoop> rotating file");
        AudioFiles_WriteHeader(&SwiftErrors, &FsFile, WAVFile.SampleRate);
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

      AudioFiles_WriteHeader(&SwiftErrors, &FsFile, WAVFile.SampleRate);
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
        HAL_GPIO_WritePin(ADC_EN_GPIO_Port, ADC_EN_Pin, GPIO_PIN_RESET);

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
      AudioFiles_WriteHeader(&SwiftErrors, &FsFile, WAVFile.SampleRate);
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
        HAL_GPIO_WritePin(ADC_EN_GPIO_Port, ADC_EN_Pin, GPIO_PIN_RESET);

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

      AudioFiles_WriteHeader(&SwiftErrors, &FsFile, WAVFile.SampleRate);
      AudioFiles_CloseFile(&SwiftErrors, &FsFile);
      BufferWritesPerFile = WAVFile.FileSize;

      if ((SwiftSchedule.CurrentRecordNumber + 1) != SwiftSchedule.NumberOfRecords)
        SetNextStartTime(&SwiftSchedule);

      xEventGroupSetBits(xEventGroup, BLU_LED_SIGNAL);
      __HAL_RCC_I2C1_CLK_ENABLE();
      AudioCodec_SleepMode(&hi2c1);
      __HAL_RCC_I2C1_CLK_DISABLE();
      HAL_GPIO_WritePin(ADC_EN_GPIO_Port, ADC_EN_Pin, GPIO_PIN_RESET);

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
      /* Day-skip check */
      if (ShouldSkipToday())
      {
        WriteSystemLog("<ISchedule Task> Day-skip: skipping today");
        continue;
      }

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

  /* Enable ADC power supply (PB15 active-high) before codec init */
  HAL_GPIO_WritePin(ADC_EN_GPIO_Port, ADC_EN_Pin, GPIO_PIN_SET);
  HAL_Delay(10);  /* Allow codec to power up before I2C communication */

  /* Step 1: Configure codec registers (reset, wake, ASI format, channel enables).
     PWR_CFG is NOT written yet — the codec PLL needs BCLK/FSYNC present when it
     powers up. Writing PWR_CFG before SAI clocks start causes PLL lock failure. */
  __HAL_RCC_I2C1_CLK_ENABLE();
  MX_I2C1_Init();
  /* Gain from flash: CODEC_SETTINGS_OFFSET+3, 0.5dB/step, 0–95 (ADC3120 max 84=42dB) */
  uint8_t codecGain    = *(__IO uint8_t *)(SETTINGS_BASE_ADDRESS + CODEC_SETTINGS_OFFSET + 3);
  if (codecGain == 0xFF) codecGain = 66;  /* default 33dB if unprogrammed */

  /* Mic bias from flash: CODEC_SETTINGS_OFFSET+13 (0x00=off, 0x10=2.5V, 0x18=3.3V) */
  uint8_t codecMicBias = *(__IO uint8_t *)(SETTINGS_BASE_ADDRESS + CODEC_SETTINGS_OFFSET + 13);
  if (codecMicBias == 0xFF) codecMicBias = 0x10;  /* default 2.5V if unprogrammed */

  HAL_StatusTypeDef codecRet = (HAL_StatusTypeDef)AudioCodec_Initialize(&hi2c1, codecGain, codecMicBias);
  (void)codecRet;

  /* Step 2: Start SAI DMA — this enables BCLK and FSYNC from the SAI master.
     
     GPDMA linked-list mode: We use two nodes (first half / second half).
     Both generate TC events, but there's no HT flag. We register a custom
     DMA XferCpltCallback that uses a toggle to distinguish half vs complete.
     
     IMPORTANT: We CANNOT use HAL_SAI_Receive_DMA() because it overwrites
     the pre-configured linked-list nodes. Instead we start DMA manually. */
  MX_GPDMA1_Init();
  /* Reconfigure PLL2 for the requested sample rate, then init SAI.
     PLL2 = 98.304 MHz (M=1, N=32, P=4) covers all supported rates exactly.
     MckOverSampling is set per-rate in MX_SAI1_Init to keep MCKDIV within 6-bit limit. */
  {
    uint32_t fs = WAVFile.SampleRate;
#ifdef FORCE_SAMPLE_RATE
    fs = FORCE_SAMPLE_RATE;
#endif
    if (fs != 8000 && fs != 12000 && fs != 16000 && fs != 24000 && fs != 32000 && fs != 48000 && fs != 96000 && fs != 192000)
      fs = 32000;
    WAVFile.SampleRate = fs;  /* keep WAV header in sync with actual rate */
    MX_SAI1_ReconfigPLL2(fs);
    MX_SAI1_Init(fs);
  }

  /* Reset toggle and warmup skip before starting DMA */
  g_dmaHalfToggle = 0;
  g_dmaWarmupSkip = 2;  /* discard first 2 signals (1 full buffer of pre-lock silence) */

  /* Register custom DMA callback directly — bypasses SAI HAL's internal callbacks
     which don't work correctly with two-node linked-list (no HT interrupt). */
  hdma_sai1_a.XferCpltCallback = DMA_SAI_XferCpltCallback;

  /* Start DMA manually — HAL_SAI_Receive_DMA would overwrite our node config.
     HAL_DMAEx_List_Start_IT starts the linked-list DMA with interrupts enabled. */
  HAL_StatusTypeDef dmaRx = HAL_DMAEx_List_Start_IT(&hdma_sai1_a);

  /* Update SAI handle state for consistency */
  hsai_BlockA1.State = HAL_SAI_STATE_BUSY_RX;

  /* Enable SAI DMA request and start SAI */
  SAI1_Block_A->CR1 |= SAI_xCR1_DMAEN;
  __HAL_SAI_ENABLE(&hsai_BlockA1);

  /* Step 3: Now that BCLK/FSYNC are running, power up the codec PLL + ADC.
     Give the SAI a moment to start clocking before the codec PLL tries to lock. */
  HAL_Delay(5);
  AudioCodec_PowerUp(&hi2c1, codecMicBias);
  __HAL_RCC_I2C1_CLK_DISABLE();
  (void)dmaRx;
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
  * @brief  On-demand flash log dump.  If dump_flash_logs.cmd exists on SD,
  *         dump all flash log entries to a timestamped .out file, erase flash
  *         log area, then disable SDMMC1 clock.
  */
static void CheckAndDumpFlashLogs(void)
{
  static FIL DumpFile;
  UINT bw;
  char StringBuffer[200];
  uint8_t StringLength;
  char read_buffer[MAX_STRING_LENGTH];
  char filename[40];
  char datetime[16];

  /* Check if the trigger file exists */
  if (f_stat("0:/dump_flash_logs.cmd", NULL) != FR_OK)
    return;

  /* Delete trigger file so it doesn't fire again */
  f_unlink("0:/dump_flash_logs.cmd");

  /* Build output filename: Flash_logs_YYYYMMDD_hhmmss.out */
  RTC_ReturnDateTimeFileName(datetime);
  snprintf(filename, sizeof(filename), "0:/Flash_logs_%s.out", datetime);

  if (f_open(&DumpFile, filename, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    return;

  /* Write header */
  StringLength = snprintf(StringBuffer, sizeof(StringBuffer), "=== Flash Log Dump ===\r\n");
  f_write(&DumpFile, StringBuffer, StringLength, &bw);

  RTC_ReturnDateTimeString(StringBuffer);
  f_write(&DumpFile, StringBuffer, 32, &bw);

  StringLength = snprintf(StringBuffer, sizeof(StringBuffer), "\r\n======================\r\n\r\n");
  f_write(&DumpFile, StringBuffer, StringLength, &bw);

  /* Walk through all flash log entries */
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
      f_write(&DumpFile, StringBuffer, StringLength, &bw);
    }

    uint32_t entry_size = 16 + ((string_length + 15) & ~15);
    current_addr += entry_size;
  }

  f_close(&DumpFile);
  Flash_ErasePage(FLASH_USER_START_ADDR, FLASH_USER_END_ADDR);
}

/**
  * @brief  Check if today should be skipped based on day-skip interval.
  * @retval 1 = skip today, 0 = record today
  */
static uint8_t ShouldSkipToday(void)
{
  if (g_daySkip <= 1) return 0;  /* 0 or 1 = record every day */

  /* If start date is default (Year=0 → 1/1/2000), use DOY-based spacing */
  if (SwiftSchedule.StartDate.Year == 0)
  {
    return (DSTSettings.Current_DOY % g_daySkip != 1) ? 1 : 0;
  }

  /* User-configured start date: use counter */
  g_daySkipCounter++;
  if (g_daySkipCounter >= g_daySkip)
  {
    g_daySkipCounter = 0;
    return 0;  /* Record today */
  }
  return 1;  /* Skip today */
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
        /* Day-skip check: if today should be skipped, go back to standby */
        if (ShouldSkipToday())
        {
          WriteSystemLog("<ISchedule> Day-skip: skipping today");
          continue;
        }

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
      HAL_GPIO_WritePin(ADC_EN_GPIO_Port, ADC_EN_Pin, GPIO_PIN_RESET);
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
    uint8_t codecGain    = *(__IO uint8_t *)(SETTINGS_BASE_ADDRESS + CODEC_SETTINGS_OFFSET + 3);
    if (codecGain == 0xFF) codecGain = 66;
    uint8_t codecMicBias = *(__IO uint8_t *)(SETTINGS_BASE_ADDRESS + CODEC_SETTINGS_OFFSET + 13);
    if (codecMicBias == 0xFF) codecMicBias = 0x10;
    AudioCodec_Initialize(&hi2c1, codecGain, codecMicBias);

    RTC_AddTimeAndSetAlarmA();

    MX_GPDMA1_Init();
    {
      uint32_t fs = WAVFile.SampleRate;
#ifdef FORCE_SAMPLE_RATE
      fs = FORCE_SAMPLE_RATE;
#endif
      if (fs != 8000 && fs != 12000 && fs != 16000 && fs != 24000 && fs != 32000 && fs != 48000 && fs != 96000 && fs != 192000)
        fs = 32000;
      WAVFile.SampleRate = fs;
      MX_SAI1_ReconfigPLL2(fs);
      MX_SAI1_Init(fs);
    }
    g_dmaHalfToggle = 0;
    g_dmaWarmupSkip = 2;
    hdma_sai1_a.XferCpltCallback = DMA_SAI_XferCpltCallback;
    HAL_DMAEx_List_Start_IT(&hdma_sai1_a);
    hsai_BlockA1.State = HAL_SAI_STATE_BUSY_RX;
    SAI1_Block_A->CR1 |= SAI_xCR1_DMAEN;
    __HAL_SAI_ENABLE(&hsai_BlockA1);
    HAL_Delay(5);
    AudioCodec_PowerUp(&hi2c1, codecMicBias);
    __HAL_RCC_I2C1_CLK_DISABLE();

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
        AudioFiles_WriteHeader(&SwiftErrors, &VoiceMemoFile, WAVFile.SampleRate);
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
  * @brief  Button hold task — waits for raw press event, confirms 1.5s hold,
  *         then sets BUTTON_SIGNAL. Ignores short presses.
  */
void ButtonTask(void const *argument)
{
  (void)argument;
  for (;;)
  {
    /* Wait for falling edge from ISR */
    xEventGroupWaitBits(xEventGroup, BUTTON_PRESS_SIGNAL, pdTRUE, pdFALSE, osWaitForever);

    /* Wait 1.5s then check if button is still held (PC6 low = pressed) */
    osDelay(1500);
    if (HAL_GPIO_ReadPin(PUSHBUTTON_GPIO_Port, PUSHBUTTON_Pin) == GPIO_PIN_RESET)
    {
      /* Solid blue for 1s as confirmation feedback, then blink blue */
      StatusLED_SolidBlueLED();
      osDelay(1000);
      xEventGroupSetBits(xEventGroup, BUTTON_SIGNAL);
    }
    /* Discard any additional press events that accumulated during the hold wait */
    xEventGroupClearBits(xEventGroup, BUTTON_PRESS_SIGNAL);
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

  /* Try to log the error to flash — safe even if flash logging isn't fully
     initialized, WriteFlashNextEntry checks internally */
  WriteFlashNextEntry("*** Error_Handler called ***\r\n");

  /* Use RTC backup register DR2 as a retry counter (survives NVIC_SystemReset).
     After 5 failed attempts, enter permanent red-blink standby. */
  uint32_t retryCount = 0;

  /* Only read/write backup registers if RTC clock is likely running
     (check if the magic number in DR0 is set — means RTC was initialized) */
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) == 0x32F2U)
  {
    retryCount = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
    retryCount++;
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, retryCount);

    if (retryCount < 5)
    {
      __enable_irq();
      NVIC_SystemReset();
      while (1) {}  /* Should not reach here */
    }

    /* 5 failures — reset counter for next power cycle */
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, 0);
  }

  /* Permanent error: blink RED LED via raw register access */
  RCC->AHB2ENR1 |= RCC_AHB2ENR1_GPIOCEN;
  volatile uint32_t dummy = RCC->AHB2ENR1;
  (void)dummy;

  GPIOC->MODER   = (GPIOC->MODER & ~(3UL << (1 * 2))) | (1UL << (1 * 2));
  GPIOC->OTYPER &= ~(1UL << 1);
  GPIOC->OSPEEDR &= ~(3UL << (1 * 2));

  while (1)
  {
    GPIOC->BSRR = (1UL << 1);
    for (volatile uint32_t i = 0; i < 200000; i++) {}
    GPIOC->BSRR = (1UL << (1 + 16));
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
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(VBUS_SCALED_GPIO_Port, &GPIO_InitStruct);

  if (HAL_GPIO_ReadPin(VBUS_SCALED_GPIO_Port, VBUS_SCALED_Pin) == GPIO_PIN_SET)
  {
    /* ---- USB MODE ---- */
    SystemClock_Config_USB();
    MX_RTC_Init();
    MX_TIM6_Init();       /* HAL timebase - USB stack uses HAL_Delay */
    MX_GPIO_Init_USB();
    MX_ADC1_Init();   /* init once — GetBatteryVoltage just does start/poll/read/stop */
    /* Pre-populate battery cache before USB enumerates so first GET_REPORT has a valid value */
    g_CachedBatteryVoltage = GetBatteryVoltage();
    MX_USB_DEVICE_Init();
  }
  else
  {
    /* ---- RECORDING MODE ---- */
    SystemClock_Config();
    MX_RTC_Init();
    MX_TIM6_Init();       /* HAL timebase so HAL_Delay works */
    MX_GPIO_Init();
    MX_ADC1_Init();   /* init once — GetBatteryVoltage just does start/poll/read/stop */
    MX_SDMMC1_SD_Init();
    MX_LPTIM1_Init();

    StatusLED_Initialize();

    HAL_Delay(1000);  /* Allow voltage to settle for accurate battery read */

    xEventGroup = xEventGroupCreate();

    osThreadDef(InitializeFATFS, InitializeFATTask, osPriorityIdle, 0, 768);
    InitializeFATFSHandle = osThreadCreate(osThread(InitializeFATFS), NULL);

    osKernelStart();
  }

  /* USB mode main loop — process deferred flash writes and refresh battery cache */
  while (1)
  {
    /* Check for USB unplug: suspend callback sets g_usbSuspendTick.
       Wait 500ms for VBUS to fully discharge through R20+R21 (115k),
       then confirm VBUS_SCALED is low before resetting. */
    if (g_usbSuspendTick != 0 && (HAL_GetTick() - g_usbSuspendTick) >= 500)
    {
      if (HAL_GPIO_ReadPin(VBUS_SCALED_GPIO_Port, VBUS_SCALED_Pin) == GPIO_PIN_RESET)
      {
        NVIC_SystemReset();
      }
      g_usbSuspendTick = 0;  /* Not a real unplug (host suspend) — clear and continue */
    }

    USB_HID_ProcessFlash();

    /* Refresh battery voltage cache every ~10 seconds.
       GetBatteryVoltage() uses HAL_Delay + blocking ADC — must NOT be called
       from USB interrupt context. Update here; GET_REPORT reads g_CachedBatteryVoltage. */
    static uint32_t lastBattTick = 0;
    uint32_t now = HAL_GetTick();
    if (g_CachedBatteryVoltage < 0.0f || (now - lastBattTick) >= 10000)
    {
      g_CachedBatteryVoltage = GetBatteryVoltage();
      lastBattTick = now;
    }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif
