/* FlashLogging.h — SwiftOSH (STM32U545RET6Q)
 * Ported from SwiftOne FlashLogging.h
 *
 * STM32U545 has 512KB single-bank flash with 8KB pages.
 * Logging area: 0x08080000 is BEYOND the 512KB boundary, so we use
 * a smaller area within flash. We reserve pages 48-62 (0x08060000-0x0807DFFF)
 * for logging, keeping page 63 (0x0807E000) for settings.
 *
 * NOTE: STM32U545 only has 512KB flash (0x08000000-0x0807FFFF).
 * The original SwiftOne used 0x08080000+ which was in Bank 2 of the 1MB/2MB L4R9.
 * Here we use 0x08060000-0x0807DFFF (120KB, pages 48-62).
 */
#ifndef __FlashLogging_H
#define __FlashLogging_H

#include "stm32u5xx_hal.h"

typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR_ERASE,
    FLASH_ERROR_WRITE,
    FLASH_ERROR_VERIFY,
    FLASH_ERROR_SIZE
} FlashStatus_t;

/* STM32U545: 512KB single-bank, 8KB pages (64 pages total) */
#define FLASH_PAGE_SIZE_BYTES         8192

/* Maximum string length for a single log entry */
#define MAX_STRING_LENGTH             512

/* USB HID settings page — MUST NOT be erased by logging code */
#define FLASH_SETTINGS_PAGE_START     0x0807E000
#define FLASH_SETTINGS_PAGE_END       0x0807FFFF

/* Logging area: pages 48-62 (0x08060000 - 0x0807DFFF = 120KB) */
#define FLASH_USER_START_ADDR         ((uint32_t)0x08060000)
#define FLASH_USER_END_ADDR           ((uint32_t)0x0807DFFF)

FlashStatus_t Flash_Init(void);
FlashStatus_t Flash_EraseSinglePage(uint32_t PageAddress);
FlashStatus_t Flash_ErasePage(uint32_t StartAddr, uint32_t EndAddr);
FlashStatus_t Flash_WriteData(uint32_t Address, const uint8_t *pData, uint32_t DataLength);
FlashStatus_t Flash_ReadString(uint32_t Address, char *pData, uint32_t MaxLength);
uint8_t Flash_IsEmpty(uint32_t Address, uint32_t Length);
uint32_t Flash_FindNextFreeAddress(uint32_t StartAddr, uint32_t EndAddr);
FlashStatus_t WriteFlashNextEntry(const char *inputString);

#endif /* __FlashLogging_H */
