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

/**
  * @brief  Write data to flash at SETTINGS_BASE_ADDRESS + offset.
  *         Data is written in 128-bit (16-byte / quadword) chunks on STM32U5.
  * @param  buffer  Pointer to source data (must be 16-byte aligned content)
  * @param  offset  Byte offset from SETTINGS_BASE_ADDRESS
  * @param  size    Number of quadwords (16-byte units) to write
  * @retval 1 on success
  */
uint8_t WriteToFlash(uint8_t *buffer, uint32_t offset, uint16_t size)
{
  uint32_t Address = offset + SETTINGS_BASE_ADDRESS;
  uint32_t *dataPointer = (uint32_t *)buffer;

  HAL_FLASH_Unlock();

  for (int i = 0; i < size; i++)
  {
    /* STM32U5 programs in quadwords (128 bits = 16 bytes = 4 x uint32_t) */
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, Address + (16 * i),
                       (uint32_t)&dataPointer[4 * i]);
    /* Note: STM32U5 HAL_FLASH_Program takes the address of the data for
       QUADWORD programming, not the data value itself. */
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

  HAL_FLASH_Unlock();

  /* Clear error flags */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks     = FLASH_BANK_1;
  EraseInitStruct.Page      = 63;   /* 0x0807E000 / 0x2000 = page 63 */
  EraseInitStruct.NbPages   = 1;

  HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);

  HAL_FLASH_Lock();
  return 1;
}
