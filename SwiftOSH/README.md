# SwiftOSH — STM32U545RET6Q Firmware

Port of the SwiftOne audio recorder firmware from STM32L4R9 (IAR/EWARM) to
STM32U545RET6Q (GCC / STM32CubeIDE).

## What's ported

- **FreeRTOS** (CMSIS-RTOS v1 wrapper) with tickless idle and Stop-2 low-power
- **USB Custom HID** — same 251-byte report descriptor as SwiftOne, same report IDs,
  all OUT report handlers implemented (flash writes, RTC set, DST, bootloader jump)
- **SAI1** master-RX with circular DMA for TLV320ADC3120 audio codec (I2S, 16-bit stereo)
- **I2C1** (PB6/PB7) for codec configuration at address 0x4D
- **SDMMC1** 4-bit interface (PC8-PC12, PD2) with card-detect on PC13
- **LPTIM1** pushbutton long-press detection (~1.75 s at 32.768 kHz LSE)
- **RTC** with Alarm A/B for recording schedule, date/time set from USB HID
- **ADC1** battery voltage measurement (PA0 via resistor divider)
- **LED** control: RED (PC1), GREEN (PA1), BLUE (PA2)
- **Clock tree**: HSE 12.288 MHz → PLL1 ~160 MHz SYSCLK, PLL2 for SAI1 audio clock,
  HSI48 + CRS for USB mode, LSE 32.768 kHz for RTC/LPTIM
- **WriteToFlash** — flash settings storage (page 63, quadword programming for STM32U5)
- **AudioCodec** — TLV320ADC3120 I2C driver (init, sleep, wake)
- **SwiftSettings** — reads codec config, WAV attributes, schedule, DST, device info from flash
- **RTC helpers** — `RTC_SetDateTime()`, `RTC_SetDSTActiveFlag()`, `RTC_ReadDateTime()`,
  `RTC_ReturnDateTimeFileName()`, `RTC_ReturnDateString()`, `isClockWorking()`,
  schedule alarm management (`RTC_InitializeScheduleAlarms`, `SetNextStartTime/StopTime`)
- **FatFS R0.12c** — full middleware integration with exFAT, LFN, RTOS-reentrant semaphores,
  DMA-based SD card I/O (`sd_diskio.c`), BSP SD driver (`bsp_driver_sd.c`),
  `get_fattime()` from RTC for file timestamps
- **AudioFiles** — SD card file/directory management: mount (`AudioFiles_MountSDCard`),
  numbered recording directories, date-stamped day directories, timestamped WAV file
  creation with 46-byte header, WAV header write-back on close, config text file generation
- **FreeRTOS task bodies** — InitializeFATTask (SD mount, codec init, settings read, battery
  check, schedule alarm init), RecordLoopTask (DMA buffer f_write, WAV header/close on stop),
  StandbyTask (directory/file creation, codec wake, DMA start, schedule stop-time alarm),
  ErrorTask, BlinkLEDTask
- **FlashLogging** — system log to internal flash (pages 48-62, quadword programming for STM32U5),
  flash log dump to SD on boot (`WriteSystemFlashLogToSD`), debug file writing
- **StatusLED** — LED pattern abstraction via LPTIM2 interrupt-driven GPIO toggling
  (blink red, green, blue, combinations, solid modes, low-battery mode);
  uses STM32U5 per-channel LPTIM API (`HAL_LPTIM_OC_ConfigChannel`, `LPTIM_CHANNEL_1`)
- **LPModes** — low-power mode helpers: Sleep fallback, Stop 2 entry with wake-up flag clearing
  (`PWR_WAKEUP_ALL_FLAG`, `PWR_FLAG_SBF`),
  full clock restore after Stop 2 (HSE + PLL1 + PLL2 for SAI1)
- **DST event handling** — daylight savings time evaluation and RTC clock adjustment in
  record/standby tasks, filename suffix switching based on DST state, Alarm B 2AM trigger
- **Voice memo recording** — hall-effect triggered short recording to VoiceMemos directory,
  2-minute timeout via RTC Alarm A, codec init/DMA capture/file close cycle
- **Full InitializeScheduleTask** — complete schedule state machine with END_OF_DAY handling
  for arbitrary schedules, midnight recording resume, date range validation, button override
  to continuous mode
- **Error_Handler recovery** — retry counter in RTC backup register DR2, auto-reboot up to
  5 times, red+blue LED blink on permanent failure

## What's NOT ported (yet)

- SD card firmware update (`SD_FW_Update.c`)

## Dependencies

The project requires three external dependencies that are NOT checked into the repo:

| Dependency | Version | Source | Location in tree |
|---|---|---|---|
| STM32U5 HAL/CMSIS | V1.7.0 | STM32CubeU5 firmware package | `Drivers/STM32U5xx_HAL_Driver/`, `Drivers/CMSIS/` |
| FreeRTOS Kernel | V10.3.1 | [GitHub](https://github.com/FreeRTOS/FreeRTOS-Kernel) (tag `V10.3.1-kernel-only`) | `Middlewares/Third_Party/FreeRTOS/Source/` |
| STM32 USB Device Library | v2.11.3 | [GitHub](https://github.com/STMicroelectronics/stm32_mw_usb_device) (tag `v2.11.3`) | `Middlewares/ST/STM32_USB_Device_Library/` |

The STM32Cube_FW_U5_V1.7.0 package does NOT include FreeRTOS or the classic USB Device
Library (ST moved to ThreadX/USBX for U5). These must be cloned separately.

The CMSIS-RTOS v1 wrapper (`cmsis_os.c`/`cmsis_os.h`) is also not in the FreeRTOS kernel
repo. Both files must be copied from `SwiftOne_FW/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/`
into `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/`. The ST-modified `cmsis_os.h`
includes FreeRTOS headers (`FreeRTOS.h`, `task.h`, `queue.h`, `semphr.h`, `event_groups.h`,
`timers.h`); do NOT use the bare ARM CMSIS template from `Drivers/CMSIS/RTOS/Template/`
as it lacks these includes and will fail to compile.

### Setup

1. Install the STM32CubeU5 firmware package (via STM32CubeMX or STM32CubeIDE package manager)
2. Edit `setup_links.bat` — set `CUBE_U5_DIR` to your STM32Cube_FW_U5 install path
   (default: `C:\Users\ptcha\STM32Cube\Repository\STM32Cube_FW_U5_V1.7.0`)
3. Run `setup_links.bat` from an Administrator command prompt (creates directory junctions
   for HAL and CMSIS)
4. Run `setup_middleware.bat` (clones FreeRTOS kernel and USB Device Library from GitHub)
5. Copy CMSIS-RTOS v1 wrapper files into `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/`:
   - `cmsis_os.h` from `../SwiftOne_FW/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/`
   - `cmsis_os.c` from `../SwiftOne_FW/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/`

## Building

### Option A: STM32CubeIDE (v1.17.0+)
1. Run the setup scripts above to create symlinks and clone middleware
2. Import: `File → Import → Existing Projects into Workspace`, point at `SwiftOSH/`
3. Build All — the `.cproject` has source entry exclusions so only the needed HAL/FreeRTOS/USB
   files are compiled (matches the Makefile's `C_SOURCES` list)

The `.cproject` is pre-configured with:
- MCU target: STM32U545RETxQ, Cortex-M33, FPv5-SP-D16 hard float
- C standard: gnu11, optimization: -Os
- Defines: `STM32U545xx`, `USE_HAL_DRIVER`, `USE_FULL_LL_DRIVER`, `DEBUG`
- All include paths matching the Makefile
- Linker script: `STM32U545RETXQ_FLASH.ld` with GC sections
- Source entries with exclusion filters for unused HAL modules, FreeRTOS ports, and USB classes

**HAL configuration notes** (`Core/Inc/stm32u5xx_hal_conf.h`):
- `USE_RTOS` must be `0` — the STM32U5 HAL enforces this with a `#error` directive
- `HAL_GPDMA_MODULE_ENABLED` must be disabled — `stm32u5xx_hal_gpdma.h` does not exist
  in HAL V1.7.0 (GPDMA support was added in a later HAL version; V1.7.0 uses
  `stm32u5xx_hal_dma.h` via `HAL_DMA_MODULE_ENABLED` instead)
- `USE_HAL_SAI_REGISTER_CALLBACKS` must be `1U` — required for `HAL_SAI_RegisterCallback()`
  used in audio DMA setup

**STM32L4 → STM32U5 porting gotchas** (lessons learned):
- RCC peripheral clock names changed in HAL V1.7.0:
  - `RCC_PERIPHCLK_SDMMC` (not `SDMMC1`), `RCC_PERIPHCLK_ADCDAC` (not `ADC`),
    `RCC_PERIPHCLK_ICLK` (not `USB`) — USB clock goes through ICLK intermediate clock
  - Struct members: `SdmmcClockSelection` (not `Sdmmc1ClockSelection`),
    `IclkClockSelection` (not `UsbClockSelection`)
  - Source defines: `RCC_SDMMCCLKSOURCE_PLL1`, `RCC_ICLK_CLKSOURCE_HSI48`
- USB DRD clock macros: `__HAL_RCC_USB_FS_CLK_ENABLE()` / `_DISABLE()` (not `__HAL_RCC_USB_CLK_*`)
- USB Device Library v2.11.3 `OutEvent` callback: `int8_t (*)(uint8_t event_idx, uint8_t state)` —
  access full report buffer via `((USBD_CUSTOM_HID_HandleTypeDef *)hUsbDeviceFS.pClassData)->Report_buf`
- DMA: STM32U5 GPDMA does NOT support `DMA_CIRCULAR` as a Mode value — use `DMA_NORMAL`.
  Circular transfers require linked-list DMA (`stm32u5xx_hal_dma_ex.c` must be compiled).
  The SAI HAL driver references `HAL_DMAEx_List_Start_IT` internally
- SDMMC: `SDMMC_InitTypeDef.Transceiver` field only exists when `USE_SD_TRANSCEIVER != 0` —
  omit it on STM32U545
- ADC: `HAL_ADCEx_Calibration_Start()` takes 3 args: `(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)`
- LPTIM: `HAL_LPTIM_SetOnce_Start_IT()` takes `(hlptim, Channel)` — period set via
  `LPTIM_InitTypeDef.Period`, compare via `HAL_LPTIM_OC_ConfigChannel()` with
  `LPTIM_OC_ConfigTypeDef.Pulse`
- PWR wake-up flags: `PWR_FLAG_WUF1`–`WUF5` → `PWR_WAKEUP_FLAG1`–`FLAG8`;
  `PWR_FLAG_SB` → `PWR_FLAG_SBF`; use `PWR_WAKEUP_ALL_FLAG` to clear all at once
- LPTIM: `OutputPolarity` removed from `LPTIM_InitTypeDef` — use per-channel
  `LPTIM_OC_ConfigTypeDef` via `HAL_LPTIM_OC_ConfigChannel()`;
  `HAL_LPTIM_PWM_Start_IT()` / `HAL_LPTIM_PWM_Stop()` now require a `Channel` arg
- RTC DST macros (`__HAL_RTC_DAYLIGHT_SAVING_TIME_ADD1H` / `SUB1H`) expand to
  `do{...}while(0);` — always use braces `{ }` around if/else branches calling them
- FatFS: with exFAT + LFN enabled, compile ONLY `option/unicode.c` — do NOT compile
  `option/ccsbcs.c` (duplicate `ff_convert`/`ff_wtoupper` symbols)

### Option B: Makefile
```bash
# Edit CUBE_U5_DIR in Makefile to point to your STM32CubeU5 firmware package
make
make flash   # requires STM32_Programmer_CLI
```

## Pin mapping (from Hardware_Desc.txt)

| Function       | Pin  | Peripheral     |
|----------------|------|----------------|
| VBAT_SCALED    | PA0  | ADC1_CH5       |
| GREEN_LED      | PA1  | GPIO OUT       |
| BLUE_LED       | PA2  | GPIO OUT       |
| SAI1_BCLK      | PA8  | SAI1_SCK_A     |
| SAI1_WCLK      | PA9  | SAI1_FS_A      |
| SAI1_DOUT      | PA10 | SAI1_SD_A      |
| USB_DM         | PA11 | USB DRD FS     |
| USB_DP         | PA12 | USB DRD FS     |
| VBUS_SCALED    | PA15 | GPIO IN        |
| SD_EN_bar      | PB4  | GPIO OUT       |
| VBAT_MONITOR   | PB5  | GPIO           |
| I2C1_SCL       | PB6  | I2C1           |
| I2C1_SDA       | PB7  | I2C1           |
| SAI1_MCLK      | PB8  | SAI1_MCLK_A    |
| ADC_EN         | PB15 | GPIO OUT       |
| HALL_SWITCH    | PC0  | EXTI0          |
| RED_LED        | PC1  | GPIO OUT       |
| PS/SYNC        | PC2  | GPIO OUT (OD)  |
| Pushbutton     | PC6  | LPTIM1_ETR     |
| SDMMC1.D0-D3  | PC8-11| SDMMC1        |
| SDMMC1.CLK    | PC12 | SDMMC1         |
| SDMMC1.DETECT | PC13 | EXTI13         |
| SDMMC1.CMD    | PD2  | SDMMC1         |
