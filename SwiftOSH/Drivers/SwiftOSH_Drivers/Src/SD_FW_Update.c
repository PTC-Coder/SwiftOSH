/**
  ******************************************************************************
  * @file    SD_FW_Update.c
  * @brief   SD Card Firmware Update — SwiftOSH (STM32U545RET6Q)
  *
  *          Ported from SwiftOne SD_FW_Update.c with STM32U5 adaptations:
  *          - 8KB pages, dual-bank (Bank2 starts at 0x08040000)
  *          - 128-bit (quadword) programming via FLASH_NSCR/NSSR/NSKEYR
  *          - No SRAM3; image buffer in lower SRAM1
  *          - Flash updater runs directly from flash with reverse page
  *            order (highest page first, page 0 last) so the function's
  *            own code pages stay intact as long as possible
  *          - Final reset via hand-assembled RAM trampoline
  ******************************************************************************
  */
#include "SD_FW_Update.h"
#include "stm32u5xx_hal.h"
#include "FlashLogging.h"
#include "StatusLED.h"
#include "ff.h"
#include <string.h>

/* Intel HEX record types */
#define IHEX_DATA               0x00
#define IHEX_EOF                0x01
#define IHEX_EXT_SEGMENT_ADDR   0x02
#define IHEX_START_SEGMENT_ADDR 0x03
#define IHEX_EXT_LINEAR_ADDR    0x04
#define IHEX_START_LINEAR_ADDR  0x05

#define HEX_LINE_BUFFER_SIZE    600

/* STM32U5 flash register addresses (nonsecure) */
#define FLASH_BASE_REG    0x40022000U
#define FLASH_NSKEYR_ADDR (FLASH_BASE_REG + 0x08U)
#define FLASH_NSSR_ADDR   (FLASH_BASE_REG + 0x20U)
#define FLASH_NSCR_ADDR   (FLASH_BASE_REG + 0x28U)

/* NSSR bits */
#define FLASH_NSSR_BSY    (1U << 16)

/* NSCR bits */
#define FLASH_NSCR_PG     (1U << 0)
#define FLASH_NSCR_PER    (1U << 1)
#define FLASH_NSCR_BKER   (1U << 11)
#define FLASH_NSCR_STRT   (1U << 16)
#define FLASH_NSCR_LOCK   (1U << 31)
#define FLASH_NSCR_PNB_SHIFT  3

/* Bank 2 base address */
#define FLASH_BANK2_BASE  0x08040000U

/* ---- Intel HEX parsing helpers ---- */
static uint8_t hex_nibble(char c)
{
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
  return 0xFF;
}

static uint8_t hex_byte(const char *h)
{
  return (uint8_t)((hex_nibble(h[0]) << 4) | hex_nibble(h[1]));
}

static uint16_t hex_word(const char *h)
{
  return (uint16_t)((hex_byte(h) << 8) | hex_byte(h + 2));
}

/* ---- Intel HEX file parser ---- */
static FW_Update_Status_t parse_hex_file(FIL *file, uint8_t *buffer,
                                          uint32_t buf_size,
                                          uint32_t *image_size)
{
  char line[HEX_LINE_BUFFER_SIZE];
  uint32_t ext_addr = 0;
  uint32_t max_addr = 0;
  uint8_t eof_seen = 0;

  memset(buffer, 0xFF, buf_size);

  while (f_gets(line, sizeof(line), file) != NULL)
  {
    uint32_t len = strlen(line);
    if (len < 11) continue;

    char *p = line;
    while (*p && *p != ':') p++;
    if (*p != ':') continue;
    p++;

    uint8_t byte_count = hex_byte(p);     p += 2;
    uint16_t address   = hex_word(p);     p += 4;
    uint8_t rec_type   = hex_byte(p);     p += 2;

    /* Verify checksum */
    uint8_t cksum = byte_count + (address >> 8) + (address & 0xFF) + rec_type;
    for (uint8_t i = 0; i < byte_count; i++)
      cksum += hex_byte(p + i * 2);
    cksum += hex_byte(p + byte_count * 2);
    if (cksum != 0) {
      WriteFlashNextEntry("FW Update: Checksum error\r\n");
      return FW_UPDATE_HEX_ERROR;
    }

    switch (rec_type)
    {
    case IHEX_DATA:
    {
      uint32_t full_addr = ext_addr + address;
      if (full_addr < FW_APP_START_ADDR) {
        WriteFlashNextEntry("FW Update: Addr below app start\r\n");
        return FW_UPDATE_ADDR_ERROR;
      }
      /* Skip settings page */
      if (full_addr >= FW_SETTINGS_PAGE_ADDR &&
          full_addr < (FW_SETTINGS_PAGE_ADDR + FW_SETTINGS_PAGE_SIZE)) {
        p += byte_count * 2;
        break;
      }
      uint32_t offset = full_addr - FW_APP_START_ADDR;
      /* Skip data beyond buffer */
      if ((offset + byte_count) > buf_size) {
        p += byte_count * 2;
        break;
      }
      if ((full_addr + byte_count) > max_addr)
        max_addr = full_addr + byte_count;
      for (uint8_t i = 0; i < byte_count; i++) {
        uint32_t ba = full_addr + i;
        if (ba >= FW_SETTINGS_PAGE_ADDR &&
            ba < (FW_SETTINGS_PAGE_ADDR + FW_SETTINGS_PAGE_SIZE))
          continue;
        buffer[offset + i] = hex_byte(p + i * 2);
      }
      p += byte_count * 2;
      break;
    }
    case IHEX_EOF:
      eof_seen = 1;
      break;
    case IHEX_EXT_SEGMENT_ADDR:
      ext_addr = (uint32_t)hex_word(p) << 4;
      p += 4;
      break;
    case IHEX_EXT_LINEAR_ADDR:
      ext_addr = (uint32_t)hex_word(p) << 16;
      p += 4;
      break;
    case IHEX_START_SEGMENT_ADDR:
    case IHEX_START_LINEAR_ADDR:
      p += byte_count * 2;
      break;
    default:
      WriteFlashNextEntry("FW Update: Unknown record type\r\n");
      return FW_UPDATE_HEX_ERROR;
    }
    if (eof_seen) break;
  }

  if (!eof_seen) {
    WriteFlashNextEntry("FW Update: No EOF record\r\n");
    return FW_UPDATE_HEX_ERROR;
  }
  if (max_addr == 0) {
    WriteFlashNextEntry("FW Update: No data records\r\n");
    return FW_UPDATE_HEX_ERROR;
  }

  *image_size = max_addr - FW_APP_START_ADDR;
  return FW_UPDATE_OK;
}

/* ------------------------------------------------------------------ */
/* Flash updater — runs from flash with reverse page order.           */
/* Self-contained: no HAL calls, no library calls.                    */
/* Only uses shifts for division (no / or % to avoid __aeabi_uidiv).  */
/* ------------------------------------------------------------------ */
static void __attribute__((noinline, used))
flash_updater(uint8_t *src, uint32_t image_size)
{
  volatile uint32_t *nskeyr = (volatile uint32_t *)FLASH_NSKEYR_ADDR;
  volatile uint32_t *nssr   = (volatile uint32_t *)FLASH_NSSR_ADDR;
  volatile uint32_t *nscr   = (volatile uint32_t *)FLASH_NSCR_ADDR;

  /* Unlock flash */
  *nskeyr = 0x45670123U;
  *nskeyr = 0xCDEF89ABU;
  while (*nssr & FLASH_NSSR_BSY) {}
  *nssr = *nssr;  /* Clear error flags */

  /* Calculate total pages needed across both banks.
     Bank 1: pages 0-31 (0x08000000 - 0x0803FFFF, 256KB)
     Bank 2: pages 0-31 (0x08040000 - 0x0807FFFF, 256KB)
     Settings page = Bank 2 page 31 (0x0807E000) — skip it.
     Process in REVERSE order so our own code pages survive longest. */

  /* Erase and program Bank 2 pages (31 down to 0), then Bank 1 (31 down to 0) */
  /* Bank 2 first (reverse) */
  for (int32_t page = 31; page >= 0; page--)
  {
    uint32_t page_addr = FLASH_BANK2_BASE + ((uint32_t)page << 13);

    /* Skip settings page */
    if (page_addr == FW_SETTINGS_PAGE_ADDR) continue;

    uint32_t buf_offset = page_addr - FW_APP_START_ADDR;
    /* Skip pages beyond image */
    if (buf_offset >= image_size && page_addr >= FW_APP_START_ADDR) continue;

    /* Erase: BKER=1 (bank 2), PNB=page, PER=1, STRT=1 */
    *nscr = 0;
    *nscr = FLASH_NSCR_PER | ((uint32_t)page << FLASH_NSCR_PNB_SHIFT) | FLASH_NSCR_BKER;
    *nscr |= FLASH_NSCR_STRT;
    while (*nssr & FLASH_NSSR_BSY) {}
    *nssr = *nssr;

    /* Program in 16-byte (quadword) chunks */
    for (uint32_t off = 0; off < FW_FLASH_PAGE_SIZE; off += 16)
    {
      uint32_t abs_off = buf_offset + off;
      if (abs_off >= image_size) break;

      /* Check if all 0xFF (skip erased data) */
      uint8_t allFF = 1;
      for (uint32_t j = 0; j < 16; j++) {
        if (abs_off + j < image_size && src[abs_off + j] != 0xFF) {
          allFF = 0;
          break;
        }
      }
      if (allFF) continue;

      /* Set PG bit */
      *nscr = FLASH_NSCR_PG;

      /* Write 4 words (128 bits) to flash address */
      volatile uint32_t *dst = (volatile uint32_t *)(page_addr + off);
      for (uint32_t w = 0; w < 4; w++) {
        uint32_t word = 0xFFFFFFFFU;
        for (uint32_t b = 0; b < 4; b++) {
          uint32_t idx = abs_off + (w << 2) + b;
          if (idx < image_size) {
            if (b == 0) word = 0;
            word |= ((uint32_t)src[idx]) << (b << 3);
          }
        }
        dst[w] = word;
      }
      while (*nssr & FLASH_NSSR_BSY) {}
      *nscr &= ~FLASH_NSCR_PG;
      *nssr = *nssr;
    }
  }

  /* Bank 1 (reverse) */
  for (int32_t page = 31; page >= 0; page--)
  {
    uint32_t page_addr = FW_APP_START_ADDR + ((uint32_t)page << 13);
    uint32_t buf_offset = page_addr - FW_APP_START_ADDR;
    if (buf_offset >= image_size) continue;

    /* Erase: BKER=0 (bank 1), PNB=page, PER=1, STRT=1 */
    *nscr = 0;
    *nscr = FLASH_NSCR_PER | ((uint32_t)page << FLASH_NSCR_PNB_SHIFT);
    *nscr |= FLASH_NSCR_STRT;
    while (*nssr & FLASH_NSSR_BSY) {}
    *nssr = *nssr;

    /* Program */
    for (uint32_t off = 0; off < FW_FLASH_PAGE_SIZE; off += 16)
    {
      uint32_t abs_off = buf_offset + off;
      if (abs_off >= image_size) break;

      uint8_t allFF = 1;
      for (uint32_t j = 0; j < 16; j++) {
        if (abs_off + j < image_size && src[abs_off + j] != 0xFF) {
          allFF = 0;
          break;
        }
      }
      if (allFF) continue;

      *nscr = FLASH_NSCR_PG;
      volatile uint32_t *dst = (volatile uint32_t *)(page_addr + off);
      for (uint32_t w = 0; w < 4; w++) {
        uint32_t word = 0xFFFFFFFFU;
        for (uint32_t b = 0; b < 4; b++) {
          uint32_t idx = abs_off + (w << 2) + b;
          if (idx < image_size) {
            if (b == 0) word = 0;
            word |= ((uint32_t)src[idx]) << (b << 3);
          }
        }
        dst[w] = word;
      }
      while (*nssr & FLASH_NSSR_BSY) {}
      *nscr &= ~FLASH_NSCR_PG;
      *nssr = *nssr;
    }
  }

  /* Lock flash */
  *nscr |= FLASH_NSCR_LOCK;

  /* System reset via SCB->AIRCR */
  volatile uint32_t *aircr = (volatile uint32_t *)0xE000ED0CU;
  *aircr = (0x5FAU << 16) | (1U << 2);
  while (1) {}
}

/* ------------------------------------------------------------------ */
/* Public entry point                                                 */
/* ------------------------------------------------------------------ */
FW_Update_Status_t FW_Update_CheckAndApply(float battery_voltage)
{
  static FIL fil;
  FRESULT fres;
  FW_Update_Status_t status;
  uint32_t image_size = 0;

  uint8_t *fw_buffer = (uint8_t *)FW_RAM_BUFFER_ADDR;

  /* Scan SD root for any .hex file */
  DIR dir;
  FILINFO fno;
  char hex_path[4 + FW_HEX_FILENAME_MAX_LEN];
  uint8_t found = 0;

  fres = f_opendir(&dir, "0:/");
  if (fres != FR_OK) return FW_UPDATE_NO_FILE;

  while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0) {
    if (fno.fattrib & AM_DIR) continue;
    uint32_t nlen = strlen(fno.fname);
    if (nlen >= 5 &&
        (fno.fname[nlen-4] == '.') &&
        (fno.fname[nlen-3] == 'h' || fno.fname[nlen-3] == 'H') &&
        (fno.fname[nlen-2] == 'e' || fno.fname[nlen-2] == 'E') &&
        (fno.fname[nlen-1] == 'x' || fno.fname[nlen-1] == 'X')) {
      strcpy(hex_path, "0:/");
      strncat(hex_path, fno.fname, FW_HEX_FILENAME_MAX_LEN - 1);
      found = 1;
      break;
    }
  }
  f_closedir(&dir);

  if (!found) return FW_UPDATE_NO_FILE;

  fres = f_open(&fil, hex_path, FA_READ | FA_OPEN_EXISTING);
  if (fres != FR_OK) return FW_UPDATE_NO_FILE;

  WriteFlashNextEntry("FW Update: HEX file found on SD\r\n");

  /* Battery check */
  if (battery_voltage > 0.0f && battery_voltage < FW_UPDATE_MIN_BATTERY_V) {
    WriteFlashNextEntry("FW Update: Battery too low\r\n");
    f_close(&fil);
    return FW_UPDATE_LOW_BATTERY;
  }

  WriteFlashNextEntry("FW Update: Parsing HEX...\r\n");
  StatusLED_SolidBlueLED();

  /* Parse Intel HEX into RAM buffer */
  status = parse_hex_file(&fil, fw_buffer, FW_RAM_BUFFER_SIZE, &image_size);
  f_close(&fil);

  if (status != FW_UPDATE_OK) {
    WriteFlashNextEntry("FW Update: Parse error\r\n");
    f_unlink(hex_path);
    StatusLED_BlinkRedLED();
    return status;
  }

  if (image_size == 0 || image_size > FW_APP_MAX_SIZE) {
    WriteFlashNextEntry("FW Update: Invalid image size\r\n");
    f_unlink(hex_path);
    StatusLED_BlinkRedLED();
    return FW_UPDATE_TOO_LARGE;
  }

  /* Delete .hex file — point of no return approaching */
  f_unlink(hex_path);
  f_mount(NULL, "", 0);

  WriteFlashNextEntry("FW Update: Flashing. DO NOT REMOVE POWER.\r\n");

  /* Blink all 3 LEDs */
  StatusLED_BlinkRedBlueGreenLED();

  /* ==== POINT OF NO RETURN ==== */
  __disable_irq();

  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;

  for (uint32_t i = 0; i < 8; i++) {
    NVIC->ICER[i] = 0xFFFFFFFF;
    NVIC->ICPR[i] = 0xFFFFFFFF;
  }

  /* Invalidate I-cache to ensure flash reads are fresh after programming */
  volatile uint32_t *icache_cr = (volatile uint32_t *)0x40030400U;
  *icache_cr &= ~1U;  /* Disable ICACHE */

  /* Call the flash updater directly from flash.
     It processes pages in reverse order (highest first, page 0 last)
     so its own code pages survive as long as possible.
     When it finally erases its own page, the CPU stalls during the
     erase (same-bank read during erase) but resumes after completion.
     The final AIRCR reset is the last instruction. */
  flash_updater(fw_buffer, image_size);

  /* Should never reach here */
  while (1) {}
}
