/**
  ******************************************************************************
  * @file    bsp_driver_sd.c — SwiftOSH (STM32U545RET6Q)
  * @brief   SD card BSP driver, ported from SwiftOne (STM32L4)
  ******************************************************************************
  */
#include "bsp_driver_sd.h"
#include "main.h"

extern SD_HandleTypeDef hsd1;

/**
  * @brief  Initializes the SD card device.
  * @retval SD status
  */
__weak uint8_t BSP_SD_Init(void)
{
  uint8_t sd_state = MSD_OK;

  if (BSP_SD_IsDetected() != SD_PRESENT)
  {
    return MSD_ERROR_SD_NOT_PRESENT;
  }

  sd_state = HAL_SD_Init(&hsd1);

  if (sd_state == MSD_OK)
  {
    if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
    {
      sd_state = MSD_ERROR;
    }
  }

  return sd_state;
}

__weak uint8_t BSP_SD_ITConfig(void)
{
  return (uint8_t)0;
}

__weak uint8_t BSP_SD_ReadBlocks(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  uint8_t sd_state = MSD_OK;
  if (HAL_SD_ReadBlocks(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks, Timeout) != HAL_OK)
    sd_state = MSD_ERROR;
  return sd_state;
}

__weak uint8_t BSP_SD_WriteBlocks(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks, uint32_t Timeout)
{
  uint8_t sd_state = MSD_OK;
  if (HAL_SD_WriteBlocks(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks, Timeout) != HAL_OK)
    sd_state = MSD_ERROR;
  return sd_state;
}

__weak uint8_t BSP_SD_ReadBlocks_DMA(uint32_t *pData, uint32_t ReadAddr, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MSD_OK;
  if (HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK)
    sd_state = MSD_ERROR;
  return sd_state;
}

__weak uint8_t BSP_SD_WriteBlocks_DMA(uint32_t *pData, uint32_t WriteAddr, uint32_t NumOfBlocks)
{
  uint8_t sd_state = MSD_OK;
  if (HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK)
    sd_state = MSD_ERROR;
  return sd_state;
}

__weak uint8_t BSP_SD_Erase(uint32_t StartAddr, uint32_t EndAddr)
{
  uint8_t sd_state = MSD_OK;
  if (HAL_SD_Erase(&hsd1, StartAddr, EndAddr) != HAL_OK)
    sd_state = MSD_ERROR;
  return sd_state;
}

__weak uint8_t BSP_SD_GetCardState(void)
{
  return ((HAL_SD_GetCardState(&hsd1) == HAL_SD_CARD_TRANSFER) ? SD_TRANSFER_OK : SD_TRANSFER_BUSY);
}

__weak void BSP_SD_GetCardInfo(BSP_SD_CardInfo *CardInfo)
{
  HAL_SD_GetCardInfo(&hsd1, CardInfo);
}

void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd)
{
  BSP_SD_AbortCallback();
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
  BSP_SD_WriteCpltCallback();
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
  BSP_SD_ReadCpltCallback();
}

__weak void BSP_SD_AbortCallback(void) {}
__weak void BSP_SD_WriteCpltCallback(void) {}
__weak void BSP_SD_ReadCpltCallback(void) {}

__weak uint8_t BSP_SD_IsDetected(void)
{
  /* PC13 = SDMMC1.DETECT — most sockets pull low when card is inserted.
     External pull-up expected on the board (GPIO configured with no internal pull). */
  if (HAL_GPIO_ReadPin(SDCARD_DETECT_GPIO_Port, SDCARD_DETECT_Pin) == GPIO_PIN_RESET)
    return SD_PRESENT;
  else
    return SD_NOT_PRESENT;
}
