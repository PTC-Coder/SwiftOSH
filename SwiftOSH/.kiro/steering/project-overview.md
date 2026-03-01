# SwiftOSH Project Overview

SwiftOSH is a port of the SwiftOne wildlife audio recorder firmware from STM32L4R9 (IAR/EWARM) to STM32U545RET6Q (GCC/STM32CubeIDE).

## Architecture

- MCU: STM32U545RET6Q (Cortex-M33, 512KB flash, 274KB SRAM: 192K SRAM1 + 64K SRAM2 + 16K SRAM4 + 2K BKPSRAM)
- RTOS: FreeRTOS via CMSIS-RTOS v1 wrapper
- Audio: TLV320ADC3120 codec over I2C1, captured via SAI1 master-RX with DMA (linked-list circular mode on STM32U5 GPDMA)
- Storage: SDMMC1 4-bit SD card interface
- USB: Custom HID for configuration (same protocol as SwiftOne)
- Low power: Stop-2 mode with LSE-driven RTC/LPTIM wakeup

## External Dependencies

Three external dependencies are NOT in the repo and must be set up before building:

- **STM32U5 HAL/CMSIS** (V1.7.0) — directory junctions from STM32CubeU5 firmware package via `setup_links.bat`
  - `Drivers/STM32U5xx_HAL_Driver/` → junction to STM32Cube_FW_U5_V1.7.0
  - `Drivers/CMSIS/` → junction to STM32Cube_FW_U5_V1.7.0
- **FreeRTOS Kernel** (V10.3.1-kernel-only) — cloned from GitHub via `setup_middleware.bat`
  - `Middlewares/Third_Party/FreeRTOS/Source/` — kernel sources
  - `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/` — CMSIS-RTOS v1 wrapper (both `cmsis_os.c` and `cmsis_os.h` must be manually copied; the ST-modified `cmsis_os.h` includes FreeRTOS headers — do NOT use the bare ARM template from `Drivers/CMSIS/RTOS/Template/`)
- **STM32 USB Device Library** (v2.11.3) — cloned from GitHub via `setup_middleware.bat`
  - `Middlewares/ST/STM32_USB_Device_Library/`

IMPORTANT: STM32Cube_FW_U5_V1.7.0 does NOT include FreeRTOS or the classic USB Device Library. ST replaced them with ThreadX and USBX for the U5 series. These must be sourced separately.

## Directory Structure

```
SwiftOSH/
├── Core/
│   ├── Inc/           — Application headers (main.h, FreeRTOSConfig.h, USB headers,
│   │                    ffconf.h, fatfs.h, bsp_driver_sd.h, sd_diskio.h)
│   ├── Src/           — Application sources (main.c, HAL MSP, IT handlers, USB, FreeRTOS,
│   │                    fatfs.c, bsp_driver_sd.c, sd_diskio.c)
│   └── Startup/       — GCC startup assembly (startup_stm32u545retxq.s)
├── Drivers/
│   ├── STM32U5xx_HAL_Driver/ — [JUNCTION] STM32CubeU5 HAL drivers
│   ├── CMSIS/                — [JUNCTION] ARM CMSIS headers + STM32U5 device headers
│   └── SwiftOSH_Drivers/
│       ├── Inc/       — Custom driver headers (AudioFiles.h, AudioCodec.h, RTC_Swift.h, etc.)
│       └── Src/       — Custom driver implementations
├── Middlewares/
│   ├── ST/
│   │   └── STM32_USB_Device_Library/ — [CLONED] USB Device core + CustomHID class
│   └── Third_Party/
│       ├── FatFs/src/ — FatFS R0.12c library (ff.c/h, diskio, ff_gen_drv, option/)
│       └── FreeRTOS/Source/ — [CLONED] FreeRTOS V10.3.1 kernel + CMSIS_RTOS wrapper
├── Makefile           — GCC build system
├── setup_links.bat    — Creates directory junctions for HAL/CMSIS (requires admin)
├── setup_middleware.bat — Clones FreeRTOS and USB Device Library from GitHub
└── STM32U545RETXQ_FLASH.ld — Linker script
```

## STM32CubeIDE Project Configuration

The `.cproject` is configured for STM32CubeIDE v1.17.0+ managed build with:
- MCU: STM32U545RETxQ, Cortex-M33, FPv5-SP-D16 hard float
- C standard: gnu11, optimization: -Os
- Defines: STM32U545xx, USE_HAL_DRIVER, USE_FULL_LL_DRIVER, DEBUG
- Source entries with exclusion filters:
  - `Drivers/STM32U5xx_HAL_Driver` — excludes all unused HAL/LL modules (only compiles the ~27 modules matching the Makefile, including `stm32u5xx_hal_dma_ex.c` for linked-list DMA support)
  - `Middlewares` — excludes ALL FreeRTOS portable directories except `GCC/ARM_CM33_NTZ/non_secure` (the kernel repo ships ~20 portable dirs for other architectures), all heap implementations except `heap_4`, all USB classes except `CustomHID`, USB template files (`usbd_conf_template.c`, `usbd_desc_template.c`), and FatFS `option/ccsbcs.c` (conflicts with `unicode.c`)
- Linker: `STM32U545RETXQ_FLASH.ld` with GC sections enabled

## HAL Configuration (`Core/Inc/stm32u5xx_hal_conf.h`)

- `USE_RTOS` must be `0` — the STM32U5 HAL enforces this with `#error`
- `HAL_GPDMA_MODULE_ENABLED` must be disabled — `stm32u5xx_hal_gpdma.h` does not exist in HAL V1.7.0 (GPDMA support was added in a later version; V1.7.0 uses `stm32u5xx_hal_dma.h` via `HAL_DMA_MODULE_ENABLED`)
- `USE_HAL_SAI_REGISTER_CALLBACKS` must be `1U` — required for `HAL_SAI_RegisterCallback()` used in audio DMA setup

## FatFS Integration

- FatFS R0.12c with exFAT support, LFN on heap (via FreeRTOS pvPortMalloc), reentrant with CMSIS-RTOS v1 semaphores
- Build: compile `option/unicode.c` only — do NOT compile `option/ccsbcs.c` (duplicate `ff_convert`/`ff_wtoupper` symbols with exFAT+LFN enabled)
- SD card I/O uses DMA with RTOS message queue for completion signaling (sd_diskio.c)
- BSP SD driver (bsp_driver_sd.c) wraps HAL_SD calls with weak callbacks overridden by sd_diskio.c
- `get_fattime()` in fatfs.c reads RTC for file timestamps
- `AudioFiles_MountSDCard()` links the SD driver and mounts the volume — called from InitializeFATTask
- Recording flow: StandbyTask creates directories/files via AudioFiles, RecordLoopTask writes DMA buffers via f_write, WAV header written on file close

## FreeRTOS

- FreeRTOS V10.3.1 kernel (cloned from GitHub, tag `V10.3.1-kernel-only`)
- CMSIS-RTOS v1 wrapper (`cmsis_os.c`/`cmsis_os.h`) must be manually copied into `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/` — not included in the kernel repo. Use the ST-modified version that includes FreeRTOS headers (`FreeRTOS.h`, `task.h`, `queue.h`, `semphr.h`, `event_groups.h`, `timers.h`); do NOT use the bare ARM template from `Drivers/CMSIS/RTOS/Template/`
- Port: `GCC/ARM_CM33_NTZ/non_secure` for Cortex-M33 non-TrustZone
- Heap: `heap_4.c`
- Tickless idle enabled (`configUSE_TICKLESS_IDLE=1`) for Stop 2 low-power

## USB

- STM32 USB Device Library v2.11.3 (cloned from GitHub)
- CustomHID class — same 251-byte report descriptor and report IDs as SwiftOne
- STM32U545 uses USB DRD (Device/Host), not OTG
- `OutEvent` callback signature in SwiftOSH: `int8_t (*)(uint8_t *buffer)` — SwiftOne-style, NOT the v2.11.3 template `(uint8_t event_idx, uint8_t state)`. Buffer[0] is always the report ID
- `DataOut()` in `usbd_customhid.c` re-arms the OUT endpoint after calling `OutEvent` via `USBD_CUSTOM_HID_ReceivePacket(pdev)` — required so the host can send subsequent reports
- SET_REPORT (OUT reports) use a deferred flash queue: `OutEvent_FS` enqueues each report into a 16-entry circular FIFO (`g_FlashQueue`), and `USB_HID_ProcessFlash()` is called from the main loop to process one entry per call. This prevents USB host timeouts from flash erase/write latency
- Flash stores the full OUT report buffer verbatim: `[reportID][arraySize][data...]`. GET_REPORT reads raw flash and returns it unchanged. The host zeros byte[0] on both sides before comparing — so byte[0] is ignored, bytes 1–N must match exactly. `SwiftSettings` readers use `+2` or `+3` offsets to skip reportID and arraySize
- Report 2 (codec settings) always arrives first and triggers `EraseFlashSector()` which wipes the entire settings page — all other reports must follow after this erase
- Report 4 (RTC set) and report 9 (bootloader jump) are handled immediately in `OutEvent_FS`, not deferred
- Do NOT assume the host sends the full declared report size — buffer may be shorter than the HID descriptor declares
- **CRITICAL — EP0 SET_REPORT payload includes the report ID byte.** The host sends `[reportID][arraySize][data...]` as the complete EP0 data payload. Do NOT prepend the report ID from `req->wValue` — doing so shifts every byte by one and silently corrupts all written settings. In `usbd_customhid.c`: call `USBD_CtlPrepareRx(pdev, hhid->Report_buf, req->wLength)` directly into `Report_buf[0]`. The symptom of double-prepending is flash reading back with the first two bytes identical (e.g., `02 02 18 18` instead of `02 18 20 38`).

## Key Peripherals and Pin Mapping

- HSE 12.288 MHz crystal → PLL1 ~160 MHz SYSCLK
- PLL2 → SAI1 audio clock (exact 32/48 kHz sample rates)
- HSI48 + CRS (trimmed from LSE) for USB mode
- LSE 32.768 kHz for RTC and LPTIM1 (Stop-2 wakeup timer)
- LPTIM2 for LED blink timing — clock source MUST be `RCC_LPTIM2CLKSOURCE_LSE` in `SystemClock_Config()`. With LSE/128 prescaler = 256 Hz counter. `BLINK_PERIOD = 768-1` gives 3s period. Auto-reload interrupt turns LEDs ON; compare match interrupt (after `BLINK_ON_TICKS = 38-1` ≈ 150ms) turns them OFF. Both callbacks live in main.c
- LPTIM1 for Stop-2 wakeup only — NOT used for button detection
- Pushbutton: PC6 — plain EXTI falling-edge with internal pull-up (NOT LPTIM1 ETR). ISR sets `BUTTON_PRESS_SIGNAL` (bit 18); `ButtonTask` confirms 1.5s hold then sets `BUTTON_SIGNAL`
- SAI1: PA8=BCLK, PA9=WCLK, PA10=DOUT, PB8=MCLK
- I2C1: PB6=SCL, PB7=SDA (codec at 0x4D)
- SDMMC1: PC8-11=D0-D3, PC12=CLK, PD2=CMD, PC13=card detect
- USB: PA11=DM, PA12=DP
- LEDs: RED=PC1 (active-high), GREEN=PA1 (active-high), BLUE=PA2 (active-high)

## Original Firmware Reference

The original SwiftOne firmware was used as a reference during porting. The SwiftOSH codebase is now self-contained — no external SwiftOne_FW reference folder is needed.

## Current State (Bring-Up)

- Board boots and reaches main() — diagnostic LED blink confirmed working
- Two startup bugs RESOLVED (see "Bring-Up Findings" below):
  1. Linker script had 272K RAM but only 256K is contiguous → invalid stack pointer → instant fault
  2. Startup assembly was missing `.data` copy and `.bss` zero-fill → garbage in all globals
- `main()` is currently a diagnostic LED test — next step is restoring HAL_Init and peripheral init
- `HAL_InitTick()` is overridden as a no-op in main.c — prevents SysTick from being configured by HAL
- `HardFault_Handler` in `stm32u5xx_it.c` blinks RED LED (PC1) via raw register writes — essential for debugging, do NOT revert to `while(1){}`
- SVC_Handler, PendSV_Handler, SysTick_Handler are provided by the FreeRTOS CM33 port via `#define` mappings in `FreeRTOSConfig.h` — do NOT define them in `stm32u5xx_it.c`
- The startup assembly (`startup_stm32u545retxq.s`) disables SysTick, clears pending PendSV/SysTick, calls SystemInit, copies `.data`, zeros `.bss`, then calls main

## Bring-Up Findings So Far

### Boot fault root cause #1: Invalid stack pointer (RESOLVED)
The linker script defined `RAM LENGTH = 272K`, making `_estack = 0x20044000`. But the STM32U545 SRAM layout is:
- SRAM1: 192KB @ 0x20000000
- SRAM2: 64KB @ 0x20030000 (contiguous with SRAM1 → 256KB total)
- SRAM4: 16KB @ 0x28000000 (SmartRun domain, NOT contiguous)

Address 0x20040000–0x20043FFF is unmapped. The initial SP loaded from the vector table pointed into invalid memory. Every C function prologue pushes registers to the stack → immediate bus fault on the first `push` instruction.

Fix: Changed linker script to `LENGTH = 256K` so `_estack = 0x20040000` (top of contiguous SRAM1+SRAM2).

### Boot fault root cause #2: Missing .data/.bss initialization (RESOLVED)
The startup assembly (`Reset_Handler`) was missing the standard `.data` copy (flash → RAM) and `.bss` zero-fill loops. Every global variable contained random SRAM garbage. Any HAL function call would dereference garbage pointers in uninitialized handle structs → HardFault.

Fix: Added `.data` copy loop, `.bss` zero loop, and `bl SystemInit` to `Reset_Handler` before `bl main`.

### LED polarities (confirmed on hardware)
- RED (PC1): Active-high — SET = ON, RESET = OFF
- GREEN (PA1): Active-high — SET = ON, RESET = OFF
- BLUE (PA2): Active-high — SET = ON, RESET = OFF
- All three LEDs are active-high (original SwiftOne code assumed GREEN/BLUE were active-low — wrong for this board)

### SD card mount reliability (RESOLVED)
Flash log analysis revealed the SD card mount was failing repeatedly on cold boot and after Stop 2 wake-up, requiring full reboots to eventually succeed. Four issues found and fixed:

1. **No retry logic in InitializeFATTask**: The task called `AudioFiles_MountSDCard()` once and terminated on failure, forcing a full reboot. Fix: Added a 5-attempt retry loop with `HAL_SD_DeInit()` and 500ms delay between attempts.
2. **SDMMC registers lost in Stop 2**: The SDMMC peripheral registers are in the AHB2 domain which is powered down in Stop 2. `StandbyTask` and `InitializeScheduleTask` re-enabled the SDMMC clock after wake-up but never re-configured the peripheral. Fix: Added `MX_SDMMC1_SD_Init()` calls after every Stop 2 wake-up path (StandbyTask, InitializeScheduleTask case 1 and case 2).
3. **FatFS driver slot leak on retries**: `AudioFiles_MountSDCard()` called `FATFS_LinkDriver()` without unlinking first, so retries would exhaust the FatFS driver slots. Fix: Added `FATFS_UnLinkDriver(SD_Path)` before `FATFS_LinkDriver()` (note: capital L in `UnLink`).
4. **BSP_SD_IsDetected() was a stub**: Always returned `SD_PRESENT` without reading the actual detect pin. Fix: Now reads PC13 (SDMMC1.DETECT) — LOW = card present (external pull-up, switch to ground).

### SD card hot-insertion detection (RESOLVED)
PC13 was configured as `GPIO_MODE_IT_RISING_FALLING` EXTI but no callback handled it. Fix:
- Added `HAL_GPIO_EXTI_Falling_Callback` for PC13 — sets `SD_INSERTED_SIGNAL` (bit 17 in event group)
- `ErrorTask` now listens for both `ERROR_SIGNAL` and `SD_INSERTED_SIGNAL` — on card insertion, does 200ms debounce then `NVIC_SystemReset()` for a clean reboot through the full init path with retry logic

### LED blink duty cycle too long (RESOLVED)
LPTIM2 auto-reload ISR was toggling LEDs, giving 50% duty cycle over a ~3.9s period (~2s ON). Fix: Changed to pulse mode — auto-reload turns LEDs ON, new `HAL_LPTIM_CompareMatchCallback` turns them OFF after ~150ms (38 ticks at 256 Hz). `BLINK_ON_TICKS` define in StatusLED.c controls the ON duration.

### USB HID GET_REPORT not working (RESOLVED)
The SwiftOne Firmware Updater app showed all zeros (SN: 00000000000, FW: 0.0.0.0). The host app uses `GET_REPORT` control requests (not interrupt IN reports), but the SwiftOSH USB middleware had the handler behind an `#ifdef USBD_CUSTOMHID_CTRL_REQ_GET_REPORT_ENABLED` that wasn't defined. Fix: Added inline GET_REPORT handling directly in `usbd_customhid.c` Setup handler, matching the SwiftOne pattern — responds to each report ID with the appropriate data via `USBD_CtlSendData()`.

### Battery voltage reading 0V (RESOLVED)
`ADC_EN` (PB15) controls the battery voltage divider but was never driven high before ADC reads. In recording mode, `MX_GPIO_Init()` configured it as output but drove it LOW. In USB mode, `MX_GPIO_Init_USB()` didn't configure it at all. Fix:
- `GetBatteryVoltage()` now drives `ADC_EN` HIGH with 5ms settling delay before reading, then drives it LOW after to save power
- `MX_GPIO_Init_USB()` now configures `ADC_EN`, `VBAT_SCALED` (PA0 analog), and enables GPIOB clock

### What works
- Raw register LED blink — MCU boots and reaches main() reliably
- HardFault_Handler with LED blink — confirmed HardFaults visible as red LED
- Startup assembly with .data/.bss init and SystemInit
- HAL_Init, SystemClock_Config_USB (HSE + PLL + HSI48 + CRS)
- MX_RTC_Init
- USB HID enumerates properly on PC host
- USB HID GET_REPORT returns serial number, firmware version, battery voltage, and all settings
- USB HID SET_REPORT (write settings to device) — deferred flash queue implemented and verified working
- BLUE LED turns on when USB connected
- SD card mounts reliably with retry logic
- SD card hot-insertion triggers reboot and re-mount
- SDMMC re-initializes properly after Stop 2 wake-up
- LED blink shows quick ~150ms pulse every 3 seconds (BLINK_PERIOD=768 at 256 Hz)
- Battery voltage reads correctly via ADC with voltage divider enable
- Pushbutton long-press (1.5s hold) detected via EXTI + ButtonTask — solid blue 1s feedback then BUTTON_SIGNAL

### Key observations
- The FreeRTOS CM33 port (`ARM_CM33_NTZ/non_secure`) hardcodes `PendSV_Handler`, `SVC_Handler`, `SysTick_Handler` as function names directly in `portasm.c` and `port.c`. The `#define` mappings in `FreeRTOSConfig.h` are redundant for this port but must remain because the port source uses those exact CMSIS names
- The interrupt handlers in `stm32u5xx_it.c` (TIM6, EXTI, SDMMC, SAI, USB, LPTIM, RTC) are strong symbols that match vector table entries. Even when main() doesn't call them, the linker keeps them, and they pull in HAL IRQ handler functions → FreeRTOS → full dependency chain. This means ALL builds of this project have FreeRTOS handlers in the vector table
- STM32CubeProgrammer "RUNNING Program" does a jump (load SP + branch), NOT a hardware reset. Processor state from the bootloader is inherited
- Flashing erases only the sectors covered by the hex file, not a full chip erase
- ICACHE is disabled by default after reset on STM32U5 — stale cache is not a concern at boot

### Files modified during bring-up
- `Core/Src/main.c` — HAL_InitTick override, diagnostic main()
- `Core/Src/stm32u5xx_it.c` — HardFault_Handler now blinks RED LED, removed SVC/PendSV/SysTick (provided by FreeRTOS port)
- `Core/Inc/FreeRTOSConfig.h` — #define mappings restored (port needs them)
- `Core/Startup/startup_stm32u545retxq.s` — added .data copy, .bss zero-fill, SystemInit call, PendSV/SysTick clearing and SysTick disable
- `STM32U545RETXQ_FLASH.ld` — fixed RAM LENGTH from 272K to 256K

### Boot fault root cause #3: USB interrupts not firing (RESOLVED)
The startup assembly vector table had `USART2_IRQHandler` at position 62, but the STM32U545 does NOT have USART2 — that slot is reserved. This shifted every subsequent vector entry by one word, so USB_IRQHandler (position 73) was actually pointing at the CRS handler, and so on. USB interrupts were silently going to the wrong handler.

Fix: Changed position 62 from `USART2_IRQHandler` to `0` (reserved). All vectors from position 63 onward now align correctly with the hardware interrupt table.

### USB D+ pull-up stability (RESOLVED)
The USB BCDR register D+ pull-up write was unreliable immediately after enabling the USB peripheral. The host would sometimes not see the device connect.

Fix: Added `HAL_Delay(5)` in `USBD_LL_Start()` (usbd_conf.c) before writing `USB_BCDR_DPPU`. This gives the USB peripheral time to stabilize after `__HAL_PCD_ENABLE`.

### BLUE LED polarity (RESOLVED)
`MX_GPIO_Init_USB()` was writing `GPIO_PIN_RESET` to turn on the blue LED, but BLUE (PA2) is active-high (same as RED), not active-low.

Fix: Changed to `GPIO_PIN_SET` in `MX_GPIO_Init_USB()`.

### SD card not mounting in recording mode (INVESTIGATING)
Four issues found:
1. **SDMMC GPIO missing pull-ups**: Fix: Added `GPIO_PULLUP` on D0-D3 and CMD, `GPIO_SPEED_FREQ_VERY_HIGH`.
2. **Bus width set to 4-bit before init**: Fix: Changed to `SDMMC_BUS_WIDE_1B`.
3. **Clock divider too aggressive**: Fix: Changed `ClockDiv` from 0 to 2 (~20 MHz).
4. **PLL1-P not enabled**: `HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC)` returned 0. Diagnostic confirmed `SDMMC_ERROR_INVALID_PARAMETER`. Fix: Added explicit `__HAL_RCC_PLLCLKOUT_ENABLE(RCC_PLL1_DIVP)` in `MX_SDMMC1_SD_Init()`. Also added missing `PLL2ClockOut = RCC_PLL2_DIVP` to `SystemClock_Config()`.

### StatusLED polarity bugs (RESOLVED)
All LEDs are active-high. Original code assumed GREEN/BLUE were active-low. Fixed all GPIO writes in StatusLED.c, BlinkLEDTask, and LowBatteryTask.

### StatusLED LPTIM2 blink too fast (RESOLVED)
LPTIM2 kernel clock defaulted to PCLK1 (80 MHz) — blink period was 1.6ms (appeared solid). Fix: Added `RCC_LPTIM2CLKSOURCE_LSE` to `SystemClock_Config()`. Now 256 Hz counter, 3s blink cycle.

LPTIM2 MSP (`HAL_LPTIM_MspInit`) MUST have an `LPTIM2` branch that enables the clock and configures NVIC. Without it, the LPTIM2 interrupt never fires and LEDs never blink. The LPTIM2 branch does NOT configure any GPIO AF pins — LED toggling is done in software from the interrupt callbacks.

`HAL_LPTIM_CompareMatchCallback` in main.c handles the LED OFF transition for LPTIM2. `HAL_LPTIM_AutoReloadMatchCallback` handles the LED ON transition. Both check `hlptim->Instance == LPTIM2` and use explicit `GPIO_PIN_SET`/`GPIO_PIN_RESET` — never `HAL_GPIO_TogglePin`.

`BLINK_PERIOD = 768-1` (3s at 256 Hz), `BLINK_ON_TICKS = 38-1` (~150ms ON pulse).

### Pushbutton implementation (RESOLVED)
PC6 was originally attempted as LPTIM1 ETR trigger input (AF1). This approach was abandoned because: (1) configuring PC6 as AF pin means `HAL_GPIO_ReadPin` always returns 0 regardless of physical state, (2) LPTIM1 ETR is complex and unreliable for simple button detection.

Fix: PC6 configured as plain `GPIO_MODE_IT_FALLING` with `GPIO_PULLUP`. `HAL_GPIO_EXTI_Falling_Callback` handles both PC6 (button) and PC13 (SD detect). PC6 ISR sets `BUTTON_PRESS_SIGNAL` (bit 18) — a raw event that does NOT directly trigger recording stop.

`ButtonTask` waits on `BUTTON_PRESS_SIGNAL`, delays 1500ms, then reads PC6. If still low (held), it calls `StatusLED_SolidBlueLED()` for 1s feedback, then sets `BUTTON_SIGNAL` (bit 2) which the recording tasks respond to. Short presses are silently ignored. Any accumulated `BUTTON_PRESS_SIGNAL` bits are cleared after the hold check.

LPTIM1 is still initialized for Stop-2 wakeup timing — its MSP no longer configures PC6 as AF GPIO.

### STM32CubeIDE debug configuration (RESOLVED)
ST-Link debug was causing HardFaults when launched from the IDE play button. Correct settings:
- SWD frequency: 8000 kHz (not 140 kHz — too slow)
- Reset behaviour: Software system reset (not "Connect under reset")
- RTOS Kernel Awareness: FreeRTOS / ARM_CM33 (not ThreadX/cortex_m0)
- RTOS Proxy: UNCHECKED — enabling it causes "Remote replied unexpectedly to 'vMustReplyEmpty': timeout" on port 60000
`LPModes_RestoreClockAfterStop2()` lost PLL1-P after reconfiguring PLL1. Fix: Added `RCC_PERIPHCLK_SDMMC` and `PLL2ClockOut` to the wake-up PeriphClkInit.

### USB SET_REPORT flash alignment bug (RESOLVED)
All settings writes except codec and WAV file were silently failing. Root cause: `FLASH_TYPEPROGRAM_QUADWORD` on STM32U5 requires 16-byte aligned addresses — any write to a non-aligned address fails silently (returns error, no data written).

The original offsets in `GeneralDefines.h` packed blocks tightly (codec=0, clockdiv=24, wavfile=48, ...). Only offsets 0 and 48 are 16-byte aligned; all others (24, 72, 120, 168, 216) are not. This is why only codec and WAV file appeared in the flash dump after a settings write.

Fix: All offsets in `GeneralDefines.h` padded to 16-byte boundaries:
- `CODEC_SETTINGS_OFFSET` = 0 (unchanged)
- `STM32_CLOCKDIV_OFFSET` = 32 (was 24)
- `WAVFILE_ATTRIBUTES_OFFSET` = 64 (was 48)
- `SCHEDULE_STARTTIMES_OFFSET` = 96 (was 72)
- `SCHEDULE_STOPTIMES_OFFSET` = 144 (was 120)
- `LATLONG_OFFSET` = 192 (was 168)
- `DST_OFFSET` = 240 (was 216)
- `CONFIG_TEXTFILE_OFFSET` = 272 (was 248)

`swift_defaults.c` updated to match the new layout (656 bytes total, was 632). `SwiftSettings.c` uses the constants throughout — no hardcoded offsets — so it updated automatically.

### USB SET_REPORT EP0 payload double-prepend bug (RESOLVED)
All settings were being written with every byte shifted by one position, causing the host read-back verification to fail. Root cause: the `CUSTOM_HID_REQ_SET_REPORT` handler in `usbd_customhid.c` was stashing the report ID from `req->wValue` into `Report_buf[0]` and then receiving the EP0 data payload into `&Report_buf[1]`. But the host already includes the report ID as the first byte of the EP0 payload — so the report ID ended up doubled (e.g., `02 02 18 18` instead of `02 18 20 38`).

Fix: Removed the manual stash. `USBD_CtlPrepareRx` now receives directly into `Report_buf[0]` with the full `req->wLength`. The host payload `[reportID][arraySize][data...]` lands correctly at `Report_buf[0..N]`.

### Battery voltage blocking USB interrupt (RESOLVED)
`GetBatteryVoltage()` was called directly from the GET_REPORT handler inside the USB interrupt context. It calls `HAL_Delay(5)`, `MX_ADC1_Init()`, ADC calibration, and polling — all blocking. This caused USB sluggishness and broke RTC time reading (USB interrupt starved other operations).

Fix: Added `volatile float g_CachedBatteryVoltage = -1.0f` global in `main.c`. The USB mode main loop calls `GetBatteryVoltage()` every 10 seconds and stores the result. The GET_REPORT handler for report 0x08 now reads `g_CachedBatteryVoltage` instead of calling `GetBatteryVoltage()` directly. `MX_GPIO_Init_USB()` no longer drives `ADC_EN` high at init — `GetBatteryVoltage()` manages the pin itself.

### What still needs to happen
1. ~~Fix boot faults~~ DONE
2. ~~Get HAL_Init() working~~ DONE
3. ~~Get SystemClock_Config_USB() working (HSE + PLL + HSI48)~~ DONE
4. ~~Get MX_RTC_Init() working~~ DONE
5. ~~Get MX_USB_DEVICE_Init() working — USB HID enumeration on PC~~ DONE
6. ~~Restore full VBUS-detect dual-path main() (USB vs recording mode)~~ DONE
7. ~~Get SD card mounting in recording mode~~ DONE (retry logic + Stop 2 re-init)
8. ~~Get USB HID GET_REPORT working (serial number, FW version, battery, settings)~~ DONE
9. ~~Fix battery voltage ADC reading (ADC_EN not driven)~~ DONE
10. ~~Fix SD card detect (BSP_SD_IsDetected stub)~~ DONE
11. ~~Fix LED blink duty cycle (too long ON time)~~ DONE
12. ~~USB HID SET_REPORT (write settings to device)~~ DONE — deferred flash queue, 16-byte aligned offsets, verified working
13. ~~Fix LED blink period (LPTIM2 clock source)~~ DONE — 3s period at LSE/128 = 256 Hz
14. ~~Pushbutton long-press detection~~ DONE — EXTI + ButtonTask, 1.5s hold, solid blue feedback
15. Port SD firmware update (SD_FW_Update) from SwiftOne
