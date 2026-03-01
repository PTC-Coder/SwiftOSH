# SwiftOSH Coding Conventions

## Toolchain

- Compiler: arm-none-eabi-gcc (GCC for ARM)
- IDE: STM32CubeIDE v1.17.0 (Eclipse-based, managed build with .cproject)
- HAL: STM32CubeU5 HAL V1.7.0 (via directory junction from STM32Cube_FW_U5_V1.7.0)
- RTOS: FreeRTOS V10.3.1 kernel with CMSIS-RTOS v1 API (cmsis_os.h)
- USB: STM32 USB Device Library v2.11.3 (CustomHID class)
- Standard: C11 (gnu11)

## Naming Conventions

- Peripheral handles: `h<peripheral>` (e.g., `hi2c1`, `hsai_BlockA1`, `hsd1`, `hrtc`)
- GPIO pin defines: `<FUNCTION>_Pin` and `<FUNCTION>_GPIO_Port` in main.h
- FreeRTOS tasks: `<Name>Task(void const *argument)` with matching `<Name>Handle` osThreadId
- Event group bits: `<NAME>_SIGNAL` defines using bit shifts
- Driver functions: `<Module>_<Action>()` (e.g., `AudioCodec_Initialize()`, `SwiftSettings_GetWAVFileAttributes()`)

## Flash Memory Layout (STM32U545, 512KB dual-bank, 2 × 32 pages × 8KB)

- Application code: 0x08000000 – 0x0807DFFF
- Settings page (page 63): 0x0807E000 – 0x0807FFFF
  - Offsets defined in GeneralDefines.h (CODEC_SETTINGS_OFFSET, WAVFILE_ATTRIBUTES_OFFSET, etc.)
- Flash programming uses QUADWORD (128-bit / 16-byte) writes on STM32U5
- **CRITICAL: `FLASH_TYPEPROGRAM_QUADWORD` requires 16-byte aligned addresses.** Writes to non-aligned addresses fail silently (HAL returns error, no data written, no fault). All flash offset constants in `GeneralDefines.h` are padded to multiples of 16. Do NOT add new offsets that are not 16-byte aligned.

## STM32L4 → STM32U5 Porting Notes

When porting code from the original SwiftOne (STM32L4R9):

- Replace `stm32l4xx_hal.h` with `stm32u5xx_hal.h`
- DMA: STM32U5 uses GPDMA (not DMA1/DMA2). Use `GPDMA1_ChannelN` instances. Note: HAL V1.7.0 does NOT have `stm32u5xx_hal_gpdma.h` — it uses `stm32u5xx_hal_dma.h` instead. `HAL_GPDMA_MODULE_ENABLED` must be disabled in `stm32u5xx_hal_conf.h`
- DMA circular mode: STM32U5 GPDMA does NOT support `DMA_CIRCULAR` as a Mode value. Use `DMA_NORMAL` for `Init.Mode`. Circular transfers require linked-list DMA via `HAL_DMAEx_List_SetCircularMode()` and `HAL_DMAEx_List_Start_IT()` (in `stm32u5xx_hal_dma_ex.c`). The SAI HAL driver checks for linked-list mode internally and calls `HAL_DMAEx_List_Start_IT` — so `stm32u5xx_hal_dma_ex.c` MUST be compiled even if you only use `DMA_NORMAL`
- Flash: STM32U5 uses `FLASH_TYPEPROGRAM_QUADWORD` (16 bytes) instead of `FLASH_TYPEPROGRAM_DOUBLEWORD` (8 bytes). The HAL_FLASH_Program data parameter is a pointer to the data, not the data value
- SAI: SwiftOne used SAI2, SwiftOSH uses SAI1 (same block A config)
- SAI register callbacks: `HAL_SAI_RegisterCallback()` requires `#define USE_HAL_SAI_REGISTER_CALLBACKS 1U` in `stm32u5xx_hal_conf.h`
- USB: STM32U545 has USB DRD (Device/Host), not OTG. Use `USB_DRD_FS` instance and `PCD_HandleTypeDef`
- USB clock macros: Use `__HAL_RCC_USB_FS_CLK_ENABLE()` / `__HAL_RCC_USB_FS_CLK_DISABLE()` (not `__HAL_RCC_USB_CLK_ENABLE` which is for USB_OTG_FS)
- USB Device Library v2.11.3: The v2.11.3 template changed `OutEvent` to `int8_t (*)(uint8_t event_idx, uint8_t state)`, but SwiftOSH uses the SwiftOne-style signature `int8_t (*)(uint8_t *buffer)` where Buffer[0] is the report ID. Do NOT use the template signature — the full report buffer is needed to dispatch by report ID. Access the buffer directly from the `OutEvent` parameter, not via `hUsbDeviceFS.pClassData`
- RCC peripheral clocks (HAL V1.7.0 naming):
  - `RCC_PERIPHCLK_SDMMC` (not `RCC_PERIPHCLK_SDMMC1`)
  - `RCC_PERIPHCLK_ADCDAC` (not `RCC_PERIPHCLK_ADC`)
  - `RCC_PERIPHCLK_ICLK` (not `RCC_PERIPHCLK_USB`) — USB clock goes through ICLK intermediate clock
  - `SdmmcClockSelection` (not `Sdmmc1ClockSelection`)
  - `IclkClockSelection` (not `UsbClockSelection`)
  - `RCC_SDMMCCLKSOURCE_PLL1` (not `RCC_SDMMC1CLKSOURCE_PLL1`)
  - `RCC_ICLK_CLKSOURCE_HSI48` (not `RCC_USBCLKSOURCE_HSI48`)
- SDMMC: `SDMMC_InitTypeDef.Transceiver` field only exists when `USE_SD_TRANSCEIVER != 0`. STM32U545 does not define this — omit the field
- ADC calibration: `HAL_ADCEx_Calibration_Start()` takes 3 args on U5: `(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED)` — not 2 like on L4
- IAR `__root` and `#pragma location` for flash constants don't exist in GCC. Use `__attribute__((section(".flash_settings")))` or linker script placement instead
- IAR `__interwork` and `__low_level_init` have no GCC equivalent. Bootloader jump logic goes in Reset_Handler or early main()
- Unique ID register: STM32U545 at 0x0BFA0700 (not 0x1FFF7590 like L4)
- FreeRTOS port: Use `ARM_CM33_NTZ/non_secure` for Cortex-M33 non-TrustZone
- PWR wake-up flags: STM32L4 `PWR_FLAG_WUF1`–`WUF5` become `PWR_WAKEUP_FLAG1`–`FLAG8` on U5. Use `PWR_WAKEUP_ALL_FLAG` to clear all at once. `PWR_FLAG_SB` becomes `PWR_FLAG_SBF`
- LPTIM: `LPTIM_InitTypeDef.OutputPolarity` removed on U5 — output polarity is now per-channel via `LPTIM_OC_ConfigTypeDef` and `HAL_LPTIM_OC_ConfigChannel()`. `HAL_LPTIM_PWM_Start_IT()` and `HAL_LPTIM_PWM_Stop()` now require a `Channel` parameter (e.g., `LPTIM_CHANNEL_1`). Period is set via `LPTIM_InitTypeDef.Period`, compare via `LPTIM_OC_ConfigTypeDef.Pulse`. `HAL_LPTIM_SetOnce_Start_IT()` takes `(hlptim, Channel)` — not `(hlptim, Period, Compare)` like on L4
- RTC DST macros: `__HAL_RTC_DAYLIGHT_SAVING_TIME_ADD1H` / `SUB1H` expand to `do{...}while(0);` (with trailing semicolon). Always wrap if/else branches calling these macros in braces `{ }` to avoid dangling-else errors

## HAL Configuration Notes (`Core/Inc/stm32u5xx_hal_conf.h`)

- `USE_RTOS` must be `0` — the STM32U5 HAL enforces this with `#error`
- `HAL_GPDMA_MODULE_ENABLED` must be disabled (commented out) — `stm32u5xx_hal_gpdma.h` does not exist in HAL V1.7.0. V1.7.0 uses `stm32u5xx_hal_dma.h` via `HAL_DMA_MODULE_ENABLED`
- `USE_HAL_SAI_REGISTER_CALLBACKS` must be `1U` — required for `HAL_SAI_RegisterCallback()` to exist

## FatFS Build Notes

- With exFAT + LFN enabled, compile ONLY `option/unicode.c` — do NOT compile `option/ccsbcs.c` alongside it. Both define `ff_convert` and `ff_wtoupper`, causing multiple-definition linker errors. `ccsbcs.c` is excluded in both the `.cproject` and Makefile

## Dependencies Setup

External dependencies are NOT in the repo. Before building:

1. Run `setup_links.bat` (admin prompt) — creates directory junctions for HAL/CMSIS from STM32Cube_FW_U5_V1.7.0
2. Run `setup_middleware.bat` — clones FreeRTOS V10.3.1 kernel and USB Device Library v2.11.3 from GitHub
3. Copy CMSIS-RTOS v1 wrapper into `Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/`:
   - Both `cmsis_os.c` and `cmsis_os.h` — use the ST-modified version that includes FreeRTOS headers (`FreeRTOS.h`, `task.h`, `queue.h`, `semphr.h`, `event_groups.h`, `timers.h`). Do NOT use the bare ARM template from `Drivers/CMSIS/RTOS/Template/` — it lacks these includes

NOTE: STM32Cube_FW_U5_V1.7.0 does NOT include FreeRTOS or the classic USB Device Library (ST replaced them with ThreadX/USBX for U5).

## Build

### STM32CubeIDE (managed build)
- Import: `File → Import → Existing Projects into Workspace`
- The `.cproject` has source entry exclusions so only needed HAL/FreeRTOS/USB files compile
- FreeRTOS exclusions cover ALL portable directories except `GCC/ARM_CM33_NTZ/non_secure` (the kernel repo ships ~20 portable dirs for other architectures: oWatcom, WizC, Xtensa, PIC18, etc.)
- USB exclusions cover all classes except `CustomHID`, plus template files (`usbd_conf_template.c`, `usbd_desc_template.c`)
- If re-importing after symlink changes: delete project from workspace (don't delete files), then re-import

### Makefile
```bash
# Set CUBE_U5_DIR in Makefile to your STM32CubeU5 firmware package path
make            # build
make clean      # clean
make flash      # flash via ST-Link (requires STM32_Programmer_CLI)
```

## Startup Assembly Rules (`Core/Startup/startup_stm32u545retxq.s`)

CRITICAL — mistakes here cause the MCU to appear completely dead (no LEDs, no SWD, no power draw):

- The Reset_Handler MUST initialize `.data` and `.bss` before calling main(). The correct sequence is: set SP → `cpsid i` → disable SysTick → `bl SystemInit` → copy `.data` from flash to RAM → zero `.bss` → `bl main`. Omitting the `.data`/`.bss` init causes every global variable (peripheral handles, `uwTick`, `SystemCoreClock`, FreeRTOS internals) to contain random SRAM garbage, which causes HardFault on any HAL call. This was the root cause of the HAL_Init HardFault bug
- The `.data` copy uses linker symbols `_sidata` (flash source), `_sdata` / `_edata` (RAM destination). The `.bss` zero uses `_sbss` / `_ebss`. These symbols are referenced via `ldr rN, =_symbol` (literal pool), NOT via `.word` directives at file scope
- `SystemInit()` is called BEFORE `.data`/`.bss` init because it only touches peripheral registers (RCC, SCB->CPACR for FPU), not RAM variables. This is safe and ensures the FPU is enabled before any potential FP data copy
- The `.size g_pfnVectors, .-g_pfnVectors` directive MUST appear AFTER the last vector table entry, never before the `g_pfnVectors:` label. Placing it before the label gives the vector table zero size, causing the linker to discard or misplace it. Without a valid vector table at 0x08000000, the Cortex-M33 cannot boot
- The `.fpu` directive MUST match the compiler's float ABI. The project uses hard float (`-mfloat-abi=hard -mfpu=fpv5-sp-d16`), so the startup must declare `.fpu fpv5-sp-d16` — NOT `.fpu softvfp`
- `SystemInit()` in `system_stm32u5xx.c` already enables the FPU via `SCB->CPACR`. Do not rely on startup assembly alone for FPU enable — but do not contradict it either
- NEVER modify the vector table structure without verifying the IRQ numbers against the STM32U545 reference manual. Wrong vector positions cause interrupts to fire the wrong handler or Default_Handler (infinite loop)
- Do NOT place `.word` directives (e.g., `.word _sidata`, `.word _sdata`) outside of a named section at the top of the file. They end up in the default `.text` section and can be placed before the vector table at 0x08000000, corrupting the initial SP and reset vector. The Reset_Handler already references these symbols via `ldr rN, =_symbol` which generates its own literal pool entries
- The startup MUST NOT call `__libc_init_array`. This function walks the `.init_array` section to call C++ static constructors and `__attribute__((constructor))` functions. On this project (pure C, no constructors), it crashes — likely due to stale or misaligned pointers in the init_array section from the newlib/GCC runtime. The startup calls `bl SystemInit` then `bl main` directly, skipping `__libc_init_array` entirely. Do NOT re-add it
- `SystemInit()` resets all RCC clock registers back to defaults (MSI). Any GPIO clocks enabled before `SystemInit` will be disabled. If you need debug LED output in the startup, place it AFTER `bl SystemInit`

## Error_Handler Safety Rules

`Error_Handler()` can be called at ANY point during initialization — including before clocks, GPIO, RTC, or any peripheral is configured. Therefore:

- Error_Handler MUST NOT call any HAL functions that depend on initialized peripheral handles (e.g., `HAL_RTCEx_BKUPRead` with uninitialized `hrtc`)
- Error_Handler MUST NOT call `NVIC_SystemReset()` — this creates a tight reset loop that makes the MCU appear dead and prevents SWD connection
- Error_Handler MUST NOT call driver functions like `StatusLED_BlinkRedBlueLED()` that depend on LPTIM2 or GPIO being initialized
- Error_Handler SHOULD use direct register writes (RCC->AHB2ENR1, GPIOC->MODER, GPIOC->BSRR) to blink an LED, since these work regardless of HAL/peripheral state
- The current Error_Handler blinks the RED LED (PC1) using raw register access — keep it this way

## SysTick / FreeRTOS Conflict (CRITICAL)

The FreeRTOS CM33 port (`ARM_CM33_NTZ/non_secure`) hardcodes `PendSV_Handler`, `SVC_Handler`, and `SysTick_Handler` as function names directly in `portasm.c` and `port.c`. The `#define` mappings in `FreeRTOSConfig.h` (`vPortSVCHandler → SVC_Handler`, etc.) are required because the port source uses those exact CMSIS names.

This means ALL builds of this project have FreeRTOS exception handlers in the vector table, even if main() never starts the scheduler. The interrupt handlers in `stm32u5xx_it.c` pull in HAL IRQ handlers → FreeRTOS → full dependency chain.

- `HAL_InitTick()` is overridden as a no-op in main.c — SysTick is never configured by HAL
- `HardFault_Handler` in `stm32u5xx_it.c` blinks RED LED (PC1) via raw register writes — essential for debugging. Do NOT revert to `while(1){}`
- Do NOT define `SVC_Handler`, `PendSV_Handler`, or `SysTick_Handler` in `stm32u5xx_it.c` — they are provided by the FreeRTOS port. Defining them causes multiple-definition linker errors
- Do NOT remove the `#define` mappings from `FreeRTOSConfig.h` — the port needs them
- The startup assembly clears pending PendSV/SysTick and disables SysTick before calling SystemInit
- In USB mode, SysTick stays disabled — USB runs on interrupts and doesn't need HAL_Delay
- In recording mode, TIM6 is initialized as the HAL timebase (`MX_TIM6_Init()`), and SysTick is later reclaimed by FreeRTOS when `osKernelStart()` is called
- `HAL_Delay()` will NOT work until TIM6 is initialized as an alternate timebase

## Boot Faults During Bring-Up (RESOLVED)

Two independent bugs prevented the MCU from booting:

### Root cause #1: Invalid stack pointer (linker script)
The linker script defined `RAM LENGTH = 272K`, making `_estack = 0x20044000`. But only 256KB of SRAM is contiguous (SRAM1 192K + SRAM2 64K). SRAM4 (16K) is at 0x28000000, not contiguous. Address 0x20040000–0x20043FFF is unmapped. The initial SP from the vector table pointed into invalid memory → bus fault on first `push`.

Fix: `LENGTH = 256K` → `_estack = 0x20040000`.

### Root cause #2: Missing .data/.bss initialization (startup assembly)
The startup assembly jumped straight to main() without copying `.data` from flash to RAM or zeroing `.bss`. All globals contained random SRAM garbage → HardFault on any HAL call that dereferences handle structs.

Fix: Added `.data` copy loop, `.bss` zero loop, and `bl SystemInit` to `Reset_Handler`.

### LED polarities (confirmed on hardware)
- RED (PC1): Active-high — `GPIOC->BSRR = (1UL << 1)` = ON, `GPIOC->BSRR = (1UL << 17)` = OFF
- GREEN (PA1): Active-high — `GPIOA->BSRR = (1UL << 1)` = ON, `GPIOA->BSRR = (1UL << 17)` = OFF
- BLUE (PA2): Active-high — `GPIOA->BSRR = (1UL << 2)` = ON, `GPIOA->BSRR = (1UL << 18)` = OFF

Note: The ICACHE stale-cache theory was incorrect — ICACHE is disabled by default after reset on STM32U5 (EN = 0 in ICACHE_CR at boot, confirmed in RM0456).

## Newlib Syscall Stubs

- `Core/Src/syscalls.c` provides minimal stubs for `_close`, `_fstat`, `_getpid`, `_isatty`, `_kill`, `_lseek`, `_read`, `_write`
- These are required to suppress newlib linker warnings on bare-metal builds
- Do NOT remove this file

## Linker Script Notes (`STM32U545RETXQ_FLASH.ld`)

- RAM region MUST be 256K, NOT 272K or 274K. The STM32U545 has 274KB total SRAM but only 256KB is contiguous:
  - SRAM1: 192KB @ 0x20000000
  - SRAM2: 64KB @ 0x20030000 (contiguous with SRAM1)
  - SRAM4: 16KB @ 0x28000000 (SmartRun domain, completely separate address space)
  - BKPSRAM: 2KB (backup domain)
  - Using `LENGTH = 272K` or `274K` makes `_estack` point to unmapped memory (0x20044000), causing an instant bus fault on the first stack push. This was the primary root cause of the boot failure
- RAM region must be `(rw)` not `(rwx)` — execute permission on RAM triggers "LOAD segment with RWX permissions" linker warning and is unnecessary (code runs from flash)
- Do NOT add a `/DISCARD/` section that discards `libc.a(*)`, `libm.a(*)`, or `libgcc.a(*)`. These contain runtime support functions (math, memcpy, division, etc.) needed by the application code. The startup no longer calls `__libc_init_array` (it was removed because it crashes on this project), but other libc/libgcc functions are still used
- If SRAM4 is needed later (e.g., for LPDMA buffers in Stop 2), add a separate memory region: `SRAM4 (rw) : ORIGIN = 0x28000000, LENGTH = 16K` and place specific sections there with `>SRAM4`

## STM32U5 Peripheral Address Map (CRITICAL for bare-metal register access)

STM32U5 has a completely different peripheral memory map from STM32L4/F4. Do NOT use L4/F4 register addresses:

- RCC is on AHB3, NOT AHB1. Base address: `0x46020C00` (non-secure)
  - `RCC->AHB2ENR1` = `0x46020C8C` (NOT `0x4002084C` like STM32L4)
- GPIO is on AHB2. Base addresses (non-secure):
  - GPIOA: `0x42020000`, GPIOB: `0x42020400`, GPIOC: `0x42020800`
  - GPIOC->MODER = `0x42020800`, GPIOC->BSRR = `0x42020818`
- When writing bare-metal register code (e.g., in Error_Handler or startup assembly), always derive addresses from the CMSIS device header (`stm32u545xx.h`), not from memory
- In C code, always use CMSIS struct access (`RCC->AHB2ENR1`, `GPIOC->BSRR`) which resolves to correct addresses automatically. Only assembly code needs hardcoded addresses

## Option Bytes / Boot Configuration

- STM32U545 boots from the address in `NSBOOTADD0` option byte (when `nSWBOOT0=1`, `nBOOT0=1`)
- Factory default `NSBOOTADD0` value is `0x100000` which decodes to boot address `0x08000000`
- If a board has been previously programmed or option bytes modified, `NSBOOTADD0` may point elsewhere — verify in STM32CubeProgrammer Option Bytes tab
- `TZEN` must be `0` (TrustZone disabled) — if enabled, the MCU boots to a secure address and ignores non-secure flash code
- Programming is done via USB DFU (BOOT0 mode) using STM32CubeProgrammer with the hex file from `Debug/SwiftOSH.hex`

## What Still Needs Porting

- SD firmware update (SD_FW_Update)

## USB Middleware Modifications

The STM32 USB Device Library `usbd_customhid.c` has been modified in two ways:

### GET_REPORT (inline, in Setup handler)
Inline GET_REPORT handling ported from SwiftOne. Report-ID-specific responses are handled directly in the `USBD_CUSTOM_HID_Setup` function's `CUSTOM_HID_REQ_GET_REPORT` case, responding via `USBD_CtlSendData()`. Includes application-specific headers (`GeneralDefines.h`, `WriteToFlash.h`, `RTC_Swift.h`, `SwiftSettings.h`) and calls `ReadFromFlash()`, `RTC_ReadDateTime()`, `SwiftSettings_GetSTM32VersionInfo()`, `SwiftSettings_GetUniqueID()`. This is intentional — the newer middleware's `GetReport` callback doesn't pass the report ID, making it insufficient.

Battery voltage (report 0x08): returns `g_CachedBatteryVoltage` as millivolts (2 bytes LE). Do NOT call `GetBatteryVoltage()` from this handler — it uses `HAL_Delay` and blocking ADC and will stall the USB interrupt. The cached value is updated every 10 seconds from the USB mode main loop in `main.c`.

### DataOut re-arm (in DataOut handler)
`DataOut()` calls `USBD_CUSTOM_HID_ReceivePacket(pdev)` after invoking `OutEvent`. Without this, the OUT endpoint is not re-armed and the host cannot send subsequent reports.

### SET_REPORT deferred flash queue (in usbd_custom_hid_if.c)
`OutEvent_FS` enqueues each incoming OUT report into a 16-entry circular FIFO (`g_FlashQueue[FLASH_QUEUE_DEPTH]`). `USB_HID_ProcessFlash()` is called from the main loop and processes one entry per call. This prevents USB host timeouts caused by flash erase/write latency (~20ms for page erase).

Key protocol rules:
- Report 2 (codec settings) always arrives first — triggers `EraseFlashSector()` which wipes the entire settings page. All subsequent reports write to already-erased flash
- Report 4 (RTC set) and report 9 (bootloader jump) are handled immediately in `OutEvent_FS`, not queued
- Do NOT assume the host sends the full declared report size — buffer may be shorter than the HID descriptor declares. Buffer[0] is always the report ID and IS written to flash; `SwiftSettings` readers skip it with `+2` or `+3` byte offsets
- **CRITICAL — EP0 SET_REPORT payload includes the report ID byte.** The host sends `[reportID][arraySize][data...]` as the full EP0 data payload. Do NOT prepend the report ID from `req->wValue` — that causes every byte to be shifted by one, silently corrupting all written settings. In `usbd_customhid.c` SET_REPORT handler: call `USBD_CtlPrepareRx(pdev, hhid->Report_buf, req->wLength)` directly — do NOT stash `req->wValue` into `Report_buf[0]` and receive into `&Report_buf[1]`. The symptom of double-prepending is that flash reads back with the first two bytes identical (e.g., `02 02 18 18` instead of `02 18 20 38`).
