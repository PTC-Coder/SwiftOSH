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
- `WriteToFlash(buffer, offset, size)` — writes `size` quadwords (16-byte units) to flash at SETTINGS_BASE_ADDRESS + offset
- `ReadFromFlash(buffer, offset, size)` — reads `size` bytes from flash
- `EraseFlashSector()` — erases settings page 63 (0x0807E000)
- Flash programming on STM32U5 uses QUADWORD (128-bit) writes. The HAL_FLASH_Program data parameter is a pointer to the source data

### AudioCodec (AudioCodec.h / AudioCodec.c)
- `AudioCodec_Initialize(hi2c)` — configures TLV320ADC3120 via I2C using settings from flash
- `AudioCodec_SleepMode(hi2c)` — powers down ADC for low-power
- `AudioCodec_WakeFromSleep(hi2c)` — re-enables ADC
- Codec I2C address: 0x4D (7-bit), shifted to 0x9A for HAL

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
- `AudioFiles_MountSDCard(SwiftError)` — links SD driver, mounts FatFS volume
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
- Uses LPTIM2 auto-reload interrupt to toggle GPIO (no AF routing on U545 LED pins)
- All PWM start/stop calls use `LPTIM_CHANNEL_1` (STM32U5 per-channel LPTIM API)

### LPModes (LPModes.h / LPModes.c)
- `LPModes_Sleep()` — regular Sleep mode (~1-3mA), used during boot/recording
- `LPModes_EnterStop2()` — Stop 2 entry with wake-up flag clearing (uses `PWR_WAKEUP_ALL_FLAG` and `PWR_FLAG_SBF`)
- `LPModes_RestoreClockAfterStop2()` — restores HSE, PLL1, PLL2 (SAI1), SYSCLK after Stop 2
- Gated by `g_initComplete` and `g_stop2Allowed` flags in main.c

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
