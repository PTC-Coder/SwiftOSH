---
inclusion: fileMatch
fileMatchPattern: "SwiftOSH/Drivers/**"
---

# SwiftOSH Driver Development Guide

## Driver Location

Custom drivers live in `SwiftOSH/Drivers/SwiftOSH_Drivers/`:
- Headers in `Inc/`
- Sources in `Src/`

## Implemented Drivers

### WriteToFlash (WriteToFlash.h / WriteToFlash.c)
- `WriteToFlash(buffer, offset, size)` — writes data to flash at SETTINGS_BASE_ADDRESS + offset. `size` is in **doublewords (8-byte units)** for SwiftOne compatibility — `size=3` = 24 bytes written, `size=6` = 48 bytes written. Internally converts to quadword (16-byte) writes as required by STM32U5 HAL
- `ReadFromFlash(buffer, offset, size)` — reads `size` bytes from flash
- `EraseFlashSector()` — erases settings page 63 (0x0807E000, page 31 in Bank 2)
- Flash programming on STM32U5 uses QUADWORD (128-bit / 16-byte) writes. The HAL_FLASH_Program data parameter is a pointer to the source data, not the data value
- **CRITICAL: `FLASH_TYPEPROGRAM_QUADWORD` requires 16-byte aligned addresses.** Writes to non-aligned addresses fail silently. All `offset` values passed to `WriteToFlash` must be multiples of 16. The constants in `GeneralDefines.h` are all 16-byte aligned — do not add new offsets that are not
- `g_LastWriteStatus`, `g_LastEraseStatus`, `g_LastFlashError` — debug globals for checking flash operation results

### AudioCodec (AudioCodec.h / AudioCodec.c)
- `AudioCodec_Initialize(hi2c, gain)` — configures TLV320ADC3120 via I2C
  - `gain`: digital gain for CH1_CFG2, 0.5dB/step (0x00=0dB, 0x28=20dB, 0x38=28dB)
  - Gain read from flash at `CODEC_SETTINGS_OFFSET + 3`, default 0x38 if unprogrammed
- `AudioCodec_PowerUp(hi2c)` — powers up ADC, MICBIAS, PLL. **MUST be called AFTER SAI DMA starts** — codec PLL needs BCLK/FSYNC present to lock
- `AudioCodec_SleepMode(hi2c)` — powers down ADC for low-power
- `AudioCodec_WakeFromSleep(hi2c)` — re-enables ADC
- Codec I2C address: **0x4E (7-bit)**, shifted to 0x9C for HAL (ADDR pin tied high on SwiftOSH)
- Key register configuration:
  - `REG_SLEEP_CFG (0x02)`: Wake from sleep, enable AREG
  - `REG_ASI_CFG0 (0x07)`: I2S format, 16-bit word length (0x30)
  - `REG_CH1_CFG0 (0x3C)`: Single-ended input (0x20)
  - `REG_BIAS_CFG (0x3B)`: MICBIAS voltage and ADC full-scale (default 0x00 = 2.75V)
  - `REG_PWR_CFG (0x75)`: ADC power-up, MICBIAS enable, PLL enable

### RTC_Swift (RTC_Swift.h / RTC_Swift.c)
- `RTC_SetDateTime(buffer)` — sets RTC date/time from USB HID buffer (BCD format)
- `RTC_SetDSTActiveFlag(flag)` — writes DST flag to RTC backup register DR1
- `RTC_ReadDateTime(buffer, format)` — reads current date/time into buffer (BCD or BIN)
- `RTC_ReturnDateTimeFileName(buff)` — returns "YYYYMMDD_hhmmss" string for WAV filenames
- `RTC_ReturnDateString(buff)` — returns "YYYY-MM-DD" string for day directories
- `RTC_ReturnDateTimeString(buff)` — returns formatted date-time string for debug/logging
- `isClockWorking()` — checks if RTC subsecond register is advancing (coin cell present)
- `RTC_InitializeScheduleAlarms(schedule)` — sets up Alarm A/B based on schedule type
- `SetNextStartTime(schedule)` — sets Alarm A to next recording start time
- `SetNextStopTime(schedule)` — sets Alarm A to next recording stop time, advances record number
- `RTC_SetAlarmA(time)` — sets RTC Alarm A to specified time (BCD)
- `RTC_EnableAlarmB(mode)` — enables Alarm B for midnight or 2AM
- `RTC_DeactivateAlarm(alarm)` — deactivates specified RTC alarm
- DST helpers (`DST_Active`, `DST_Inactive`, `RTC_InitiateDST`) — evaluate and apply daylight savings time adjustments. Note: `__HAL_RTC_DAYLIGHT_SAVING_TIME_ADD1H`/`SUB1H` macro calls must always be wrapped in braces `{ }` when used in if/else branches

### SwiftSettings (SwiftSettings.h / SwiftSettings.c)
- Reads configuration data from flash settings page
- `SwiftSettings_GetCodecVariables()` — codec PLL, gain, bias config
- `SwiftSettings_GetWAVFileAttributes()` — file size, filename prefix, sample rate
- `SwiftSettings_GetSwiftSchedule()` — recording schedule (arbitrary/duty-cycled/continuous)
- `SwiftSettings_GetDSTSettings()` — daylight savings time parameters
- `SwiftSettings_GetSTM32VersionInfo()` / `SwiftSettings_GetUniqueID()` — device info for USB HID

### SwiftErrors (SwiftErrors.h)
- Error tracking struct for FATFS operations
- `SWIFT_ERRORS` contains calling function enum, error number, and FRESULT
- Uses `FRESULT` from ff.h for FatFS error codes

### AudioFiles (AudioFiles.h / AudioFiles.c)
- `AudioFiles_MountSDCard(SwiftError)` — links SD driver, mounts FatFS volume. Calls `FATFS_UnLinkDriver()` before `FATFS_LinkDriver()` to handle retries safely (note: capital L in `UnLinkDriver`)
- `AudioFiles_NewRecordingDirectory(WAVFile)` — creates numbered recording directory (e.g., PREFIX000)
- `AudioFiles_NewDayDirectory(label)` — creates date-stamped subdirectory (e.g., PREFIX2026-02-11)
- `AudioFiles_NewAudioFile(label, suffix, file)` — creates timestamped WAV file, seeks past 46-byte header
- `AudioFiles_WriteHeader(SwiftError, file)` — writes WAV header with correct file size and sample rate
- `AudioFiles_CloseFile(SwiftError, file)` — truncates and closes file
- `AudioFiles_GetDiskInfoString(str)` — returns formatted disk space info
- `AudioFiles_NewConfigTextFile()` — writes SwiftConfig.txt with settings from flash

### GeneralDefines (GeneralDefines.h)
- Flash offset constants for all settings blocks
- Schedule type enums (ARBITRARY, DUTYCYCLED, CONTINUOUS)
- Codec I2C address define

### USB HID Flash Queue (usbd_custom_hid_if.c)
- `USB_HID_ProcessFlash()` — deferred flash write processor, called from the main loop in USB mode. Processes one queued report per call
- `OutEvent_FS` enqueues all flash-writing reports into `g_FlashQueue[FLASH_QUEUE_DEPTH]` (16-entry circular FIFO) and returns immediately — prevents USB host timeouts from flash erase/write latency
- Report 2 (codec settings) triggers `EraseFlashSector()` then writes at `CODEC_SETTINGS_OFFSET`. The host always sends report 2 first before any other settings report
- Reports 4 (RTC set) and 9 (bootloader jump) are handled immediately in `OutEvent_FS`, not queued
- Flash write offsets per report ID (all 16-byte aligned — QUADWORD requirement):
  - Report 2: `CODEC_SETTINGS_OFFSET` (0), size=3 (24 bytes)
  - Report 12: `STM32_CLOCKDIV_OFFSET` (32), size=3
  - Report 14: `WAVFILE_ATTRIBUTES_OFFSET` (64), size=3
  - Report 16: `SCHEDULE_STARTTIMES_OFFSET` (96), size=6 (48 bytes)
  - Report 18: `SCHEDULE_STOPTIMES_OFFSET` (144), size=6
  - Report 19: `CONFIG_TEXTFILE_OFFSET` (272) + (data[1] * 48), size=6
  - Report 22: `LATLONG_OFFSET` (192), size=6
  - Report 24: `DST_OFFSET` (240), size=4 (also calls `RTC_SetDSTActiveFlag`)
- Battery voltage for GET_REPORT (report 0x08): use `g_CachedBatteryVoltage` (declared `volatile float` in `main.c`). The USB main loop refreshes it every 10 seconds via `GetBatteryVoltage()`. Never call `GetBatteryVoltage()` from interrupt context — it uses `HAL_Delay` and blocking ADC

### FlashLogging (FlashLogging.h / FlashLogging.c)
- `Flash_Init()` — clears flash error flags
- `Flash_EraseSinglePage(addr)` — erases one 8KB page with settings page protection
- `Flash_ErasePage(start, end)` — erases range of pages
- `Flash_WriteData(addr, data, len)` — writes data using QUADWORD (16-byte) programming
- `Flash_ReadString(addr, buf, maxlen)` — reads length-prefixed string from flash
- `Flash_IsEmpty(addr, len)` — checks if flash region is erased (all 0xFF)
- `Flash_FindNextFreeAddress(start, end)` — finds next empty slot for log entry
- `WriteFlashNextEntry(str)` — writes a timestamped log string to flash, auto-erases when full
- Logging area: pages 48-62 (0x08060000 - 0x0807DFFF, 120KB)
- Settings page at 0x0807E000 (page 63) is protected from erase/write

### StatusLED (StatusLED.h / StatusLED.c)
- `StatusLED_Initialize()` — configures LPTIM2 for LED blink timing (period via `Init.Period`, compare/polarity via `HAL_LPTIM_OC_ConfigChannel()` on channel 1)
- `StatusLED_BlinkRedLED()` / `BlinkGreenLED()` / `BlinkBlueLED()` — single-color slow blink
- `StatusLED_BlinkBlueGreenLED()` / `BlinkRedBlueGreenLED()` / `BlinkRedBlueLED()` — multi-color blink
- `StatusLED_SolidBlueLED()` / `SolidGreenLED()` / `SolidAllLED()` — solid LED states
- `StatusLED_AllOutputs()` — stops LPTIM2, turns all LEDs off
- `StatusLED_LowBatteryMode()` — stops LPTIM2 for manual LED control in low-battery task
- Uses LPTIM2 auto-reload interrupt to turn LEDs ON, compare match interrupt to turn them OFF after ~150ms (`BLINK_ON_TICKS = 38-1` ticks at 256 Hz). `BLINK_PERIOD = 768-1` gives 3s period at LSE/128 = 256 Hz
- LPTIM2 clock source MUST be `RCC_LPTIM2CLKSOURCE_LSE` in `SystemClock_Config()` — defaults to PCLK1 (80 MHz) otherwise, making blink period ~1.6ms (appears solid)
- `HAL_LPTIM_MspInit` MUST have an LPTIM2 branch (clock enable + NVIC only, no GPIO AF)
- `HAL_LPTIM_AutoReloadMatchCallback` and `HAL_LPTIM_CompareMatchCallback` live in main.c (not StatusLED.c) — they reference `led_blink_red/grn/blu` via `extern volatile uint8_t`
- All PWM start/stop calls use `LPTIM_CHANNEL_1` (STM32U5 per-channel LPTIM API)
- All LED GPIO writes use explicit `GPIO_PIN_SET`/`GPIO_PIN_RESET` — NEVER `HAL_GPIO_TogglePin`

### LPModes (LPModes.h / LPModes.c)
- `LPModes_Sleep()` — regular Sleep mode (~1-3mA), used during boot/recording
- `LPModes_EnterStop2()` — Stop 2 entry with wake-up flag clearing (uses `PWR_WAKEUP_ALL_FLAG` and `PWR_FLAG_SBF`)
- `LPModes_RestoreClockAfterStop2()` — restores HSE, PLL1, PLL2 (SAI1), SYSCLK after Stop 2
- Gated by `g_initComplete` and `g_stop2Allowed` flags in main.c

## SAI/DMA Audio Recording (CRITICAL)

### GPDMA Linked-List Mode for Half-Transfer Simulation
STM32U5 GPDMA does NOT support the `DMA_CIRCULAR` mode or half-transfer interrupt like STM32L4 DMA. To achieve double-buffering:

1. **Two-node linked-list**: Create two DMA nodes, each transferring half the buffer (BUFFER_SIZE_DIV2 bytes)
2. **Circular queue**: Node1 links to Node2, Node2 links back to Node1 via `HAL_DMAEx_List_SetCircularMode()`
3. **TransferEventMode**: MUST be `DMA_TCEM_EACH_LL_ITEM_TRANSFER` in BOTH the handle init AND node config — fires TC after each node completes
4. **Custom callback**: Register `hdma_sai1_a.XferCpltCallback` directly — toggle flag tracks which half completed:
   ```c
   if (g_dmaHalfToggle == 0) { signal WRITEBUFFER1; toggle = 1; }
   else { signal WRITEBUFFER2; toggle = 0; }
   ```

### SAI Configuration for 32kHz Mono
- **AudioFrequency**: 32000
- **MckOverSampling**: `SAI_MCK_OVERSAMPLING_ENABLE` — doubles MCLK period for correct Fs calculation
- **SlotActive**: `SAI_SLOTACTIVE_0` only — mono capture from slot 0 (left/ch1)
- **MonoStereoMode**: `SAI_MONOMODE` — discards slot 1 data
- **DataSize**: `SAI_DATASIZE_16` — matches codec ASI 16-bit word length
- **ClockSource**: PLL2P at 32.768 MHz (configured in `SystemClock_Config`)

### DMA Start Sequence (Order Matters!)
1. `MX_GPDMA1_Init()` — init DMA handle
2. `MX_SAI1_Init()` — configure SAI peripheral
3. Register custom DMA callback: `hdma_sai1_a.XferCpltCallback = DMA_SAI_XferCpltCallback`
4. `HAL_DMAEx_List_Start_IT(&hdma_sai1_a)` — start DMA with interrupts
5. Enable SAI DMA request: `SAI1_Block_A->CR1 |= SAI_xCR1_DMAEN`
6. Enable SAI: `__HAL_SAI_ENABLE(&hsai_BlockA1)`
7. **THEN** call `AudioCodec_PowerUp()` — codec PLL needs BCLK/FSYNC present to lock

**WARNING**: Do NOT use `HAL_SAI_Receive_DMA()` — it overwrites the two-node linked-list configuration with its own single-node setup.

### Sample Rate Flash Settings Issue
The TLV320ADC3120 uses different sample rate encoding than the old SwiftOne codec. The flash settings byte at `CODEC_SETTINGS_OFFSET + 2` contains old codec values. Currently forcing 32kHz in `SwiftSettings_GetWAVFileAttributes()` and `AudioFiles_WriteHeader()` until a translator is implemented.

## Adding New Drivers

1. Create header in `Drivers/SwiftOSH_Drivers/Inc/`
2. Create source in `Drivers/SwiftOSH_Drivers/Src/`
3. Add the .c file to `C_SOURCES` in the Makefile
4. Include path `Drivers/SwiftOSH_Drivers/Inc` is already in the Makefile INCLUDES
5. The STM32CubeIDE managed build auto-discovers new .c files under `Drivers/SwiftOSH_Drivers/` — no .cproject edit needed for custom drivers

## HAL Module Dependencies

If a new driver needs a HAL module not currently compiled (e.g., `stm32u5xx_hal_spi.c`):
1. Add it to `C_SOURCES` in the Makefile
2. Remove it from the `excluding` list in the `Drivers/STM32U5xx_HAL_Driver` source entry in `.cproject`
