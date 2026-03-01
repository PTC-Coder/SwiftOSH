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

### Battery voltage reading — ADC HAL_TIMEOUT (UNRESOLVED — investigation stopped)
`GetBatteryVoltage()` reads VBAT_SCALED (PA0 = ADC1_IN5) via ADC1. The voltage divider is enabled by `VBAT_MONITOR_EN` (PB5, active-high). `ADC_EN` (PB15) is NOT needed for battery reads — it is only needed to power the codec in recording mode.

Current implementation:
- `MX_ADC1_Init()` is called once at startup in both USB mode and recording mode `main()` paths — NOT inside `GetBatteryVoltage()`
- `GetBatteryVoltage()` does: drive PB5 HIGH → `HAL_Delay(10)` → `HAL_ADC_Start` → `HAL_ADC_PollForConversion(100ms)` → `HAL_ADC_GetValue` → `HAL_ADC_Stop` → drive PB5 LOW
- Voltage scaling: `battery_v = (raw * 3.3f / 4095.0f) * 4.298f` — divider ratio 10/(10+33) = 0.2326, multiplier = 4.298
- ADC clock: `ADC_CLOCK_ASYNC_DIV4` (160MHz/4 = 40MHz, within 5–55MHz spec per datasheet)
- ADCDAC peripheral clock source: `RCC_ADCDACCLKSOURCE_SYSCLK` in both `SystemClock_Config()` and `SystemClock_Config_USB()`
- PA0 (VBAT_SCALED) configured as `GPIO_MODE_ANALOG` in both `MX_GPIO_Init()` and `HAL_ADC_MspInit()`

**What was tried and confirmed:**
- `HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_ADCDAC)` returns 159744000 — clock IS running
- `HAL_ADC_Start()` returns HAL_OK — ADC starts successfully
- `HAL_ADC_PollForConversion()` returns HAL_TIMEOUT (poll=3) — conversion never completes, raw=0
- Diagnostic log format in code: `<ADC> clk=%lu start=%d poll=%d CR=0x%08lX ISR=0x%08lX raw=%u`
- CR/ISR register values have NOT yet been captured from hardware — needed to identify root cause
- Root cause unknown — investigation stopped before CR/ISR dump was collected

**Key STM32U5 ADC facts confirmed:**
- `HAL_ADC_Init()` exits DEEPPWD, enables ADVREGEN, and leaves ADC fully enabled (ADEN=1). Do NOT call `HAL_ADCEx_Calibration_Start()` after `HAL_ADC_Init()` — calibration requires ADEN=0 and will always return HAL_ERROR in this sequence
- STM32U5 ADC1/ADC2 have no `ADC_CLOCK_SYNC_PCLK_DIV4` — only `ADC_CLOCK_ASYNC_DIVx` constants exist
- STM32U5 ADC1 requires channels to be pre-selected in the `PCSEL` register — `HAL_ADC_ConfigChannel()` handles this automatically via `hadc->Instance->PCSEL |= (1UL << channel_number)`
- PA0 = ADC1_IN5 (confirmed in STM32U545 datasheet). PA15 = VBUS_SCALED (USB detect input), NOT an ADC pin
- `__HAL_RCC_ADCDAC_CLK_ENABLE()` does NOT exist in HAL V1.7.0 — use `__HAL_RCC_ADC12_CLK_ENABLE()` only
- The diagnostic `WriteFlashNextEntry` call is still inside `GetBatteryVoltage()` — keep it until fixed

### What still needs to happen
1. ~~Fix boot faults~~ DONE
2. ~~Get HAL_Init() working~~ DONE
3. ~~Get SystemClock_Config_USB() working (HSE + PLL + HSI48)~~ DONE
4. ~~Get MX_RTC_Init() working~~ DONE
5. ~~Get MX_USB_DEVICE_Init() working — USB HID enumeration on PC~~ DONE
6. ~~Restore full VBUS-detect dual-path main() (USB vs recording mode)~~ DONE
7. ~~Get SD card mounting in recording mode~~ DONE (retry logic + Stop 2 re-init)
8. ~~Get USB HID GET_REPORT working (serial number, FW version, battery, settings)~~ DONE
9. Fix battery voltage ADC reading — `HAL_ADC_PollForConversion` times out (IN PROGRESS)
10. ~~Fix SD card detect (BSP_SD_IsDetected stub)~~ DONE
11. ~~Fix LED blink duty cycle (too long ON time)~~ DONE
12. ~~USB HID SET_REPORT (write settings to device)~~ DONE — deferred flash queue, 16-byte aligned offsets, verified working
13. ~~Fix LED blink period (LPTIM2 clock source)~~ DONE — 3s period at LSE/128 = 256 Hz
14. ~~Pushbutton long-press detection~~ DONE — EXTI + ButtonTask, 1.5s hold, solid blue feedback
15. Port SD firmware update (SD_FW_Update) from SwiftOne
