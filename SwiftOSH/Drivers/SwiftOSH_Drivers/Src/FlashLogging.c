/* FlashLogging.c — SwiftOSH (STM32U545RET6Q)
 * Ported from SwiftOne FlashLogging.c
 *
 * Key differences from STM32L4R9:
 *   - STM32U545 has 512KB single-bank flash, 8KB pages
 *   - Flash programming uses QUADWORD (128-bit / 16-byte) writes
 *   - HAL_FLASH_Program data parameter is a pointer to the data
 *   - Logging area: 0x08060000 - 0x0807DFFF (pages 48-62, 120KB)
 *   - Settings page at 0x0807E000 (page 63) is protected
 */

#include "stm32u5xx_hal.h"
#include "FlashLogging.h"
#include <string.h>
#include <stdio.h>

FlashStatus_t Flash_Init(void)
{
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    return FLASH_OK;
}

/* Check if an address falls within the protected settings page */
static uint8_t IsProtectedAddress(uint32_t Address)
{
    uint32_t page_start = Address & ~(FLASH_PAGE_SIZE_BYTES - 1);
    return (page_start >= FLASH_SETTINGS_PAGE_START) ? 1 : 0;
}

/* Get page number for a given address (single-bank, 8KB pages) */
static uint32_t GetPage(uint32_t Address)
{
    return (Address - 0x08000000) / FLASH_PAGE_SIZE_BYTES;
}

FlashStatus_t Flash_EraseSinglePage(uint32_t PageAddress)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    if (IsProtectedAddress(PageAddress))
        return FLASH_ERROR_ERASE;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks     = FLASH_BANK_1;
    EraseInitStruct.Page      = GetPage(PageAddress);
    EraseInitStruct.NbPages   = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
        HAL_FLASH_Lock();
        return FLASH_ERROR_ERASE;
    }

    HAL_FLASH_Lock();
    return FLASH_OK;
}

FlashStatus_t Flash_ErasePage(uint32_t StartAddr, uint32_t EndAddr)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    if (IsProtectedAddress(StartAddr) || IsProtectedAddress(EndAddr))
        return FLASH_ERROR_ERASE;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    HAL_FLASH_Unlock();

    uint32_t start_page = GetPage(StartAddr);
    uint32_t end_page   = GetPage(EndAddr);

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks     = FLASH_BANK_1;
    EraseInitStruct.Page      = start_page;
    EraseInitStruct.NbPages   = end_page - start_page + 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return FLASH_ERROR_ERASE;
    }

    HAL_FLASH_Lock();
    return FLASH_OK;
}

/* Write data to flash using QUADWORD (16-byte) programming.
 * Address must be 16-byte aligned. Data is padded with 0xFF if needed. */
FlashStatus_t Flash_WriteData(uint32_t Address, const uint8_t *pData, uint32_t DataLength)
{
    if (pData == NULL || DataLength == 0)
        return FLASH_ERROR_SIZE;

    if (Address < FLASH_USER_START_ADDR || (Address + DataLength) > (FLASH_USER_END_ADDR + 1))
        return FLASH_ERROR_SIZE;

    if (IsProtectedAddress(Address))
        return FLASH_ERROR_WRITE;

    HAL_FLASH_Unlock();

    uint32_t i;
    uint8_t quadword[16];

    for (i = 0; i < DataLength; i += 16)
    {
        memset(quadword, 0xFF, 16);
        uint32_t chunk = (DataLength - i >= 16) ? 16 : (DataLength - i);
        memcpy(quadword, &pData[i], chunk);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, Address + i, (uint32_t)quadword) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return FLASH_ERROR_WRITE;
        }
    }

    HAL_FLASH_Lock();
    return FLASH_OK;
}

FlashStatus_t Flash_ReadString(uint32_t Address, char *pData, uint32_t MaxLength)
{
    uint32_t DataLength;

    if (pData == NULL || MaxLength == 0)
        return FLASH_ERROR_SIZE;

    if (Address < FLASH_USER_START_ADDR || Address >= (FLASH_USER_END_ADDR + 1))
        return FLASH_ERROR_SIZE;

    /* First 16 bytes contain the string length (stored in first 4 bytes of quadword) */
    DataLength = *((uint32_t *)Address);
    Address += 16;  /* quadword-aligned length field */

    if (DataLength == 0xFFFFFFFF || DataLength == 0 || DataLength > MaxLength - 1)
        return FLASH_ERROR_SIZE;

    for (uint32_t i = 0; i < DataLength; i++)
        pData[i] = *((uint8_t *)(Address + i));

    pData[DataLength] = '\0';
    return FLASH_OK;
}

uint8_t Flash_IsEmpty(uint32_t Address, uint32_t Length)
{
    for (uint32_t i = 0; i < Length; i++)
    {
        if (*((uint8_t *)(Address + i)) != 0xFF)
            return 0;
    }
    return 1;
}

uint32_t Flash_FindNextFreeAddress(uint32_t StartAddr, uint32_t EndAddr)
{
    uint32_t current_addr = StartAddr;
    uint32_t string_length;

    while (current_addr < EndAddr)
    {
        if (Flash_IsEmpty(current_addr, 16))
            return current_addr;

        string_length = *((uint32_t *)current_addr);

        if (string_length == 0 || string_length > MAX_STRING_LENGTH)
            return current_addr;

        /* Entry: 16-byte length field + data padded to 16-byte boundary */
        uint32_t entry_size = 16 + ((string_length + 15) & ~15);
        current_addr += entry_size;
    }

    return 0;  /* Flash full */
}

FlashStatus_t WriteFlashNextEntry(const char *inputString)
{
    FlashStatus_t status;
    char write_buffer[256];
    strncpy(write_buffer, inputString, sizeof(write_buffer) - 1);
    write_buffer[sizeof(write_buffer) - 1] = '\0';

    uint32_t string_length = strlen(write_buffer) + 1;
    uint32_t write_address;

    status = Flash_Init();
    (void)status;

    write_address = Flash_FindNextFreeAddress(FLASH_USER_START_ADDR, FLASH_USER_END_ADDR);

    if (write_address == 0 || write_address == FLASH_USER_START_ADDR)
    {
        status = Flash_ErasePage(FLASH_USER_START_ADDR, FLASH_USER_END_ADDR);
        if (status != FLASH_OK)
            return FLASH_ERROR_ERASE;
        write_address = FLASH_USER_START_ADDR;
    }

    /* Write length as a 16-byte quadword */
    uint8_t len_quad[16];
    memset(len_quad, 0xFF, 16);
    memcpy(len_quad, &string_length, 4);

    HAL_FLASH_Unlock();
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, write_address, (uint32_t)len_quad) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return FLASH_ERROR_WRITE;
    }
    HAL_FLASH_Lock();

    write_address += 16;

    /* Write string data */
    status = Flash_WriteData(write_address, (const uint8_t *)write_buffer, string_length);
    if (status != FLASH_OK)
        return FLASH_ERROR_WRITE;

    return FLASH_OK;
}
