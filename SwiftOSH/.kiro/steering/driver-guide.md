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
- `AudioCodec_Initialize(hi2c, gain, mic_bias_cfg)` — configures TLV320ADC3120 via I2C. Does NOT power up ADC — call `AudioCodec_PowerUp()` after SAI DMA starts
  - `gain`: host gain byte, 0.5dB/step (0–95). Clamped to 84 (42dB max) internally. Register value = `gain << 1` into CH1_CFG1 bits[7:1]
  - `mic_bias_cfg`: host mic bias byte — `0x00`=off, `0x10`=2.5V (default), `0x18`=3.3V
  - Gain read from flash at `CODEC_SETTINGS_OFFSET + 3`, default 66 (33dB) if unprogrammed (0xFF)
  - Mic bias read from flash at `CODEC_SETTINGS_OFFSET + 13`, default 0x10 (2.5V) if unprogrammed
- `AudioCodec_PowerUp(hi2c, mic_bias_cfg)` — powers up ADC, PLL, and optionally MICBIAS. **MUST be called AFTER SAI DMA starts** — codec PLL auto-detects BCLK/FSYNC to lock; clocks must be present
- `AudioCodec_SleepMode(hi2c)` — puts codec into sleep mode (<10µA). Wait ≥6ms before stopping BCLK
- `AudioCodec_WakeFromSleep(hi2c, mic_bias_cfg)` — wakes codec and re-enables channel 1 and power. BCLK/FSYNC must already be running
- Codec I2C address: **0x4E (7-bit)**, shifted to 0x9C for HAL (ADDR pin tied high on SwiftOSH)
- Key register configuration (TLV320ADC3120, NOT TLV320AIC3120 — completely different register map):
  - `REG_SLEEP_CFG (0x02)`: `0x81` = wake, internal AREG (required for AVDD=3.3V); `0x00` = sleep
  - `REG_ASI_CFG0 (0x07)`: `0x40` = I2S format, 16-bit word length (default is TDM `0x30` — must change)
  - `REG_BIAS_CFG (0x3B)`: MICBIAS — `0x01`=2.5V, `0x60`=3.3V (AVDD), `0x00`=off
  - `REG_CH1_CFG0 (0x3C)`: `0x20` = single-ended input IN1P/IN1M
  - `REG_CH1_CFG1 (0x3D)`: analog gain bits[7:1] — value = `gain << 1`, max 84 (42dB)
  - `REG_IN_CH_EN (0x73)`: `0x80` = enable input channel 1 only
  - `REG_ASI_OUT_CH_EN (0x74)`: `0x80` = enable ASI output channel 1 only
  - `REG_PWR_CFG (0x75)`: `0xA0` = ADC+PLL (no MICBIAS); `0xE0` = ADC+MICBIAS+PLL

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
- `AudioFiles_WriteHeader(SwiftError, file, SampleRate)` — writes WAV header with correct file size, sample rate, and byte rate. `SampleRate` is passed explicitly (Hz) — do NOT rely on flash; always pass `WAVFile.SampleRate` which has already been validated/clamped by the caller
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

### SDCardConfig (SDCardConfig.h / SDCardConfig.c)
- `SDConfig_CheckAndApply()` — scans SD root for any `.cfg` file, reads 248-byte binary settings (7 blocks: codec, clock, audio, start/stop times, location, DST), writes to settings flash. Renames to `.done` on success, `.err` on failure
- File format: each block has byte[0]=reportID, byte[1]=blockSize (multiple of 8), byte[2+]=payload
- Must run in boot sequence BEFORE settings are read from flash
- Uses `static` buffers to avoid blowing InitializeFATTask stack

### SDCardSchedule (SDCardSchedule.h / SDCardSchedule.c)
- `SDSchedule_CheckAndApply()` — scans SD root for any `.sch` file, parses plain text schedule, writes to settings flash via read-modify-write (preserves other settings)
- Supported keys: `schedule=arbitrary|dutycycle|continuous`, `start_date=YY/MM/DD`, `stop_date=YY/MM/DD`, `day_skip=N`, `slot=HH:MM-HH:MM` (up to 20), `on_minutes=N`, `off_minutes=N`, `cycles=N`
- Uses 8KB `static page_buf` for read-modify-write of entire settings page
- Renames to `.done`/`.err` after processing

### SD_FW_Update (SD_FW_Update.h / SD_FW_Update.c)
- `FW_Update_CheckAndApply(battery_voltage)` — scans SD root for `.hex` file, parses Intel HEX into RAM buffer, erases+programs app flash, resets MCU. Does NOT return on success
- STM32U5 flash: 8KB pages, dual-bank (Bank2 at 0x08040000), 128-bit quadword programming
- Flash registers: NSKEYR(+0x08), NSSR(+0x20), NSCR(+0x28) at base 0x40022000
- NSCR bits: PG=0, PER=1, PNB=[10:3], BKER=11, STRT=16, LOCK=31
- RAM buffer: 200KB in lower SRAM1 (0x20000000) — no SRAM3 on STM32U545
- Flash updater runs directly from flash with reverse page order (Bank2 page 31 down to Bank1 page 0). When it erases its own page, CPU stalls during erase but resumes after. Final reset via AIRCR
- Settings page (0x0807E000 = Bank2 page 31) is always skipped during erase+program
- Battery voltage check: refuses update below 3.2V
- .hex file deleted from SD before flashing (prevents retry loop on parse errors)
- Boot sequence position: first SD operation after mount, before SDConfig/SDSchedule/WriteDebugFile
- WARNING: needs hardware testing — flash self-erase behavior on STM32U5 single-bank-read-during-erase may differ from STM32L4

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

### Sample Rate Constraints
PLL2 is reconfigured dynamically at recording start via `MX_SAI1_ReconfigPLL2(fs)` before `MX_SAI1_Init()`. A single PLL2 config covers all supported rates (HSE = 12.288 MHz):

- `PLL2M=1, PLL2N=32, PLL2P=4` → VCO = 393.216 MHz, PLL2P output = **98.304 MHz**
- BCLK = 64 × Fs. MCKDIV = 98304000 / (Fs × 64):
  - 8kHz → MCKDIV=192 ✓, 16kHz → MCKDIV=96 ✓, 24kHz → MCKDIV=64 ✓
  - 32kHz → MCKDIV=48 ✓, 48kHz → MCKDIV=32 ✓, 96kHz → MCKDIV=16 ✓, 192kHz → MCKDIV=8 ✓

The TLV320ADC3120 supports up to 384kHz (Table 8-6). Max supported rate is **192kHz** (MCKDIV=8).

`BUFFER_SIZE = 192000` bytes (188 KB) — sized for ≥500 ms SD write window at 96 kHz, and 250 ms at 192 kHz. Both are well above SD worst-case write latency.

Any unsupported rate falls back to 32kHz. `WAVFile.SampleRate` is updated to the validated rate before `MX_SAI1_Init()` so `AudioFiles_WriteHeader()` writes the correct rate into the WAV header.

**Both `InitializeCodecAndDMA()` and `NewVoiceMemoTask()` use this pattern:**
```c
uint32_t fs = WAVFile.SampleRate;
#ifdef FORCE_SAMPLE_RATE
fs = FORCE_SAMPLE_RATE;
#endif
if (fs != 8000 && fs != 16000 && fs != 24000 && fs != 32000 && fs != 48000 && fs != 96000 && fs != 192000)
  fs = 32000;
WAVFile.SampleRate = fs;
MX_SAI1_ReconfigPLL2(fs);
MX_SAI1_Init(fs);
```

**`MX_SAI1_ReconfigPLL2` — CRITICAL: 3-step sequence required**

`HAL_RCCEx_PeriphCLKConfig` cannot reconfigure PLL2 while it is the active SAI1 clock source — it hangs waiting for `PLL2RDY` to clear. The function uses this sequence:
1. Switch SAI1 clock source to HSI16 (`RCC_SAI1CLKSOURCE_HSI`) — frees PLL2
2. Disable PLL2 via `__HAL_RCC_PLL2_DISABLE()` + poll `RCC_FLAG_PLL2RDY` until clear (HAL V1.7.0 has no `PLL2State` field — `RCC_OscInitTypeDef.PLL2.PLL2State` does not exist)
3. Reconfigure PLL2 (M=1, N=32, P=4) and switch SAI1 back to PLL2 via `HAL_RCCEx_PeriphCLKConfig` — this handles enabling PLL2 and waiting for lock internally

`SystemClock_Config` does NOT configure PLL2 at all — PLL2 boots off. `MX_SAI1_ReconfigPLL2` is the sole owner of PLL2 and always starts it fresh from the off state.

**`FORCE_SAMPLE_RATE` compile-time override** — defined (commented out) near the top of `main.c`. Uncomment and set to any supported rate to override flash settings without using the config UI. Useful for testing rates the host app doesn't yet offer.

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
