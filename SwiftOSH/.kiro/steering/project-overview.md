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
  - `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/` — CMSIS-RTOS v1 wrapper (both `cmsis_os.c` and `cmsis_os.h` copied from `SwiftOne_FW/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/`; the ST-modified `cmsis_os.h` includes FreeRTOS headers — do NOT use the bare ARM template from `Drivers/CMSIS/RTOS/Template/`)
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
- CMSIS-RTOS v1 wrapper (`cmsis_os.c`/`cmsis_os.h`) copied from `SwiftOne_FW/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/` — not included in the kernel repo. The ST-modified `cmsis_os.h` includes FreeRTOS headers (`FreeRTOS.h`, `task.h`, `queue.h`, `semphr.h`, `event_groups.h`, `timers.h`); do NOT use the bare ARM template from `Drivers/CMSIS/RTOS/Template/`
- Port: `GCC/ARM_CM33_NTZ/non_secure` for Cortex-M33 non-TrustZone
- Heap: `heap_4.c`
- Tickless idle enabled (`configUSE_TICKLESS_IDLE=1`) for Stop 2 low-power

## USB

- STM32 USB Device Library v2.11.3 (cloned from GitHub)
- CustomHID class — same 251-byte report descriptor and report IDs as SwiftOne
- STM32U545 uses USB DRD (Device/Host), not OTG
- USB Device Library v2.11.3 `OutEvent` callback signature: `int8_t (*)(uint8_t event_idx, uint8_t state)` — access full report buffer via `hUsbDeviceFS.pClassData`

## Key Peripherals and Pin Mapping

- HSE 12.288 MHz crystal → PLL1 ~160 MHz SYSCLK
- PLL2 → SAI1 audio clock (exact 32/48 kHz sample rates)
- HSI48 + CRS (trimmed from LSE) for USB mode
- LSE 32.768 kHz for RTC and LPTIM1
- LPTIM2 for LED blink timing (auto-reload interrupt, per-channel OC config via `HAL_LPTIM_OC_ConfigChannel`, `LPTIM_CHANNEL_1`)
- SAI1: PA8=BCLK, PA9=WCLK, PA10=DOUT, PB8=MCLK
- I2C1: PB6=SCL, PB7=SDA (codec at 0x4D)
- SDMMC1: PC8-11=D0-D3, PC12=CLK, PD2=CMD, PC13=card detect
- USB: PA11=DM, PA12=DP
- LEDs: RED=PC1 (active-high), GREEN=PA1 (active-low), BLUE=PA2 (active-high)

## Original Firmware Reference

The original SwiftOne firmware lives in `../SwiftOne_FW/`. When porting additional features, reference the driver implementations in `SwiftOne_FW/Drivers/Swift_Drivers/Src/`.

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
- RED (PC1): Active-high — BSRR set = ON, BSRR reset = OFF
- GREEN (PA1): Active-low — BSRR reset = ON, BSRR set = OFF
- BLUE (PA2): Active-high — BSRR set = ON, BSRR reset = OFF (same as RED, not active-low like GREEN)

### What works
- Raw register LED blink — MCU boots and reaches main() reliably
- HardFault_Handler with LED blink — confirmed HardFaults visible as red LED
- Startup assembly with .data/.bss init and SystemInit
- HAL_Init, SystemClock_Config_USB (HSE + PLL + HSI48 + CRS)
- MX_RTC_Init
- USB HID enumerates properly on PC host
- BLUE LED turns on when USB connected

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

### What still needs to happen
1. ~~Fix boot faults~~ DONE
2. ~~Get HAL_Init() working~~ DONE
3. ~~Get SystemClock_Config_USB() working (HSE + PLL + HSI48)~~ DONE
4. ~~Get MX_RTC_Init() working~~ DONE
5. ~~Get MX_USB_DEVICE_Init() working — USB HID enumeration on PC~~ DONE
6. Restore full VBUS-detect dual-path main() (USB vs recording mode)
7. Port SD firmware update (SD_FW_Update) from SwiftOne
