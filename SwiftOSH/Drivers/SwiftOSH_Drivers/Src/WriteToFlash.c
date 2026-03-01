/**
  ******************************************************************************
  * @file    WriteToFlash.c
  * @brief   Flash settings storage — SwiftOSH (STM32U545RET6Q)
  *          Ported from SwiftOne (STM32L4R9).
  *          STM32U545 has 512KB single-bank flash, 8KB pages.
  *          Settings page: page 63 at 0x0807E000.
  ******************************************************************************
  */
#include "stm32u5xx_hal.h"
#include "WriteToFlash.h"
#include "GeneralDefines.h"
#include <string.h>

/* Global debug variables to check flash status */
volatile uint32_t g_LastFlashError = 0;
volatile HAL_StatusTypeDef g_LastEraseStatus = HAL_OK;
volatile HAL_StatusTypeDef g_LastWriteStatus = HAL_OK;

/* Check if SWAP_BANK option is enabled at runtime */
static uint32_t GetSettingsBank(void)
{
  /* FLASH_OPTR_SWAP_BANK is bit 20 in FLASH->OPTR */
  if (FLASH->OPTR & FLASH_OPTR_SWAP_BANK)
    return FLASH_BANK_1;  /* Banks swapped: upper half is Bank 1 */
  else
    return FLASH_BANK_2;  /* Normal: upper half is Bank 2 */
}

/**
  * @brief  Write data to flash at SETTINGS_BASE_ADDRESS + offset.
  *         Data is written in 128-bit (16-byte / quadword) chunks on STM32U5.
  *         NOTE: size parameter is number of DOUBLEWORDS (8-byte units) for
  *         compatibility with SwiftOne. We convert to quadwords internally.
  * @param  buffer  Pointer to source data
  * @param  offset  Byte offset from SETTINGS_BASE_ADDRESS
  * @param  size    Number of doublewords (8-byte units) - for SwiftOne compatibility
  * @retval 1 on success
  */
uint8_t WriteToFlash(uint8_t *buffer, uint32_t offset, uint16_t size)
{
  uint32_t Address = offset + SETTINGS_BASE_ADDRESS;
  
  /* Convert doubleword count to byte count */
  uint32_t byteCount = size * 8;
  
  /* 16-byte aligned buffer required for STM32U5 quadword flash writes */
  uint32_t quadword[4] __attribute__((aligned(16)));
  
  HAL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
  
  /* Wait for any previous operation to complete */
  while (__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {}

  for (uint32_t i = 0; i < byteCount; i += 16)
  {
    memset(quadword, 0xFF, 16);
    uint32_t chunk = (byteCount - i >= 16) ? 16 : (byteCount - i);
    memcpy((uint8_t*)quadword, &buffer[i], chunk);
    
    g_LastWriteStatus = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, Address + i, (uint32_t)quadword);
    if (g_LastWriteStatus != HAL_OK)
    {
      g_LastFlashError = HAL_FLASH_GetError();
      HAL_FLASH_Lock();
      return 0;
    }
  }

  HAL_FLASH_Lock();
  return 1;
}

/**
  * @brief  Read data from flash at SETTINGS_BASE_ADDRESS + offset.
  * @param  buffer  Destination buffer
  * @param  offset  Byte offset from SETTINGS_BASE_ADDRESS
  * @param  size    Number of bytes to read
  * @retval 1 on success
  */
uint8_t ReadFromFlash(uint8_t *buffer, uint32_t offset, uint16_t size)
{
  uint32_t Address = offset + SETTINGS_BASE_ADDRESS;

  for (int i = 0; i < size; i++)
  {
    buffer[i] = *(__IO uint8_t *)(Address + i);
  }

  return 1;
}

/**
  * @brief  Erase the settings flash page (page 63, 0x0807E000).
  *         STM32U545: single bank, 8KB pages.
  * @retval 1 on success
  */
uint8_t EraseFlashSector(void)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PAGEError = 0;
  HAL_StatusTypeDef status;

  HAL_FLASH_Unlock();

  /* Clear error flags */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  /* STM32U545: 512KB dual-bank flash, 256KB per bank, 32 pages per bank
     0x0807E000 is page 31 in upper half - bank depends on SWAP_BANK option */
  EraseInitStruct.Banks     = GetSettingsBank();
  EraseInitStruct.Page      = 31;   /* (0x0807E000 - 0x08040000) / 0x2000 = page 31 */
  EraseInitStruct.NbPages   = 1;

  status = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
  g_LastEraseStatus = status;
  g_LastFlashError = HAL_FLASH_GetError();

  HAL_FLASH_Lock();
  
  if (status != HAL_OK || PAGEError != 0xFFFFFFFF)
  {
    return 0;
  }
  return 1;
}
