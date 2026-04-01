/**
  * @file    SD_FW_Update.h
  * @brief   SD Card Firmware Update via Intel HEX file — SwiftOSH (STM32U545)
  *
  *          On boot, checks for a .hex file on SD card root.
  *          If found, parses Intel HEX into RAM buffer, erases app flash
  *          (protecting settings page at 0x0807E000), programs new firmware,
  *          and resets.
  *
  *          STM32U545 differences from SwiftOne (STM32L4R9):
  *            - 512KB flash, dual-bank, 8KB pages (not 4KB)
  *            - Bank 1: 0x08000000-0x0803FFFF (pages 0-31)
  *            - Bank 2: 0x08040000-0x0807FFFF (pages 0-31)
  *            - 128-bit (16-byte) quadword programming (not 64-bit doubleword)
  *            - FLASH registers: NSKEYR(+0x08), NSSR(+0x20), NSCR(+0x28)
  *            - No SRAM3; RAM buffer uses lower SRAM1 (below stack)
  *            - Flash updater runs from flash (reverse page order) with
  *              RAM trampoline for final reset
  */
#ifndef __SD_FW_UPDATE_H
#define __SD_FW_UPDATE_H

#include <stdint.h>

/* Application flash region */
#define FW_APP_START_ADDR       0x08000000U
#define FW_APP_MAX_SIZE         (504U * 1024U)  /* Pages 0-62 across both banks */

/* Protected settings page — NEVER erase */
#define FW_SETTINGS_PAGE_ADDR   0x0807E000U
#define FW_SETTINGS_PAGE_SIZE   0x2000U         /* 8KB page */

/* Flash page size (STM32U545 dual-bank) */
#define FW_FLASH_PAGE_SIZE      8192U

/* RAM buffer for firmware image.
   STM32U545 has no SRAM3.  Use SRAM1 starting at 0x20000000.
   Reserve 200KB for the image — leaves 56KB for stack/heap/globals
   at the top of SRAM1+SRAM2. */
#define FW_RAM_BUFFER_ADDR      0x20000000U
#define FW_RAM_BUFFER_SIZE      (200U * 1024U)

/* Max .hex filename length */
#define FW_HEX_FILENAME_MAX_LEN  64

/* Minimum battery voltage to allow update */
#define FW_UPDATE_MIN_BATTERY_V   3.2f

/* Flash logging area */
#define FW_FLASH_LOG_START      0x08060000U
#define FW_FLASH_LOG_END        0x0807DFFFU

typedef enum {
  FW_UPDATE_OK           = 0,
  FW_UPDATE_NO_FILE      = 1,
  FW_UPDATE_LOW_BATTERY  = 2,
  FW_UPDATE_HEX_ERROR    = 3,
  FW_UPDATE_TOO_LARGE    = 4,
  FW_UPDATE_ADDR_ERROR   = 5,
  FW_UPDATE_SD_ERROR     = 6,
  FW_UPDATE_FLASH_ERROR  = 7,
} FW_Update_Status_t;

/**
  * @brief  Check for .hex file on SD card and apply firmware update.
  *         If successful, does NOT return (MCU resets).
  *         Call from InitializeFATTask after SD mount.
  * @param  battery_voltage  Current battery voltage
  * @retval FW_Update_Status_t
  */
FW_Update_Status_t FW_Update_CheckAndApply(float battery_voltage);

#endif
