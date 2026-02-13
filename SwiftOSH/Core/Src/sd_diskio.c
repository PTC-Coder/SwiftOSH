/**
  ******************************************************************************
  * @file    sd_diskio.c — SwiftOSH (STM32U545RET6Q)
  * @brief   SD Disk I/O driver (DMA + RTOS), ported from SwiftOne
  ******************************************************************************
  */
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include <string.h>

/* Private defines */
#define QUEUE_SIZE         (uint32_t) 10
#define READ_CPLT_MSG      (uint32_t) 1
#define WRITE_CPLT_MSG     (uint32_t) 2

#define SD_TIMEOUT 30 * 1000
#define SD_DEFAULT_BLOCK_SIZE 512

/* Private variables */
static volatile DSTATUS Stat = STA_NOINIT;
static osMessageQId SDQueueID = NULL;

/* Private function prototypes */
static DSTATUS SD_CheckStatus(BYTE lun);
DSTATUS SD_initialize (BYTE);
DSTATUS SD_status (BYTE);
DRESULT SD_read (BYTE, BYTE*, DWORD, UINT);
DRESULT SD_write (BYTE, const BYTE*, DWORD, UINT);
DRESULT SD_ioctl (BYTE, BYTE, void*);

const Diskio_drvTypeDef  SD_Driver =
{
  SD_initialize,
  SD_status,
  SD_read,
  SD_write,
  SD_ioctl,
};

/* Private functions */

static int SD_CheckStatusWithTimeout(uint32_t timeout)
{
  uint32_t timer = osKernelSysTick();
  while (osKernelSysTick() - timer < timeout)
  {
    if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
      return 0;
  }
  return -1;
}

static DSTATUS SD_CheckStatus(BYTE lun)
{
  Stat = STA_NOINIT;
  if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
    Stat &= ~STA_NOINIT;
  return Stat;
}

DSTATUS SD_initialize(BYTE lun)
{
  Stat = STA_NOINIT;

  if (osKernelRunning())
  {
    if (BSP_SD_Init() == MSD_OK)
    {
      Stat = SD_CheckStatus(lun);
    }

    if (Stat != STA_NOINIT)
    {
      if (SDQueueID == NULL)
      {
        osMessageQDef(SD_Queue, QUEUE_SIZE, uint16_t);
        SDQueueID = osMessageCreate(osMessageQ(SD_Queue), NULL);
      }
      if (SDQueueID == NULL)
      {
        Stat |= STA_NOINIT;
      }
    }
  }

  return Stat;
}

DSTATUS SD_status(BYTE lun)
{
  return SD_CheckStatus(lun);
}

DRESULT SD_read(BYTE lun, BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  uint32_t timer;
  osEvent event;

  if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0)
    return res;

  uint8_t ret = BSP_SD_ReadBlocks_DMA((uint32_t*)buff, (uint32_t)(sector), count);

  if (ret == MSD_OK)
  {
    event = osMessageGet(SDQueueID, SD_TIMEOUT);
    if (event.status == osEventMessage)
    {
      if (event.value.v == READ_CPLT_MSG)
      {
        timer = osKernelSysTick();
        while (osKernelSysTick() - timer < SD_TIMEOUT)
        {
          if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
          {
            res = RES_OK;
            break;
          }
        }
      }
    }
  }

  return res;
}

DRESULT SD_write(BYTE lun, const BYTE *buff, DWORD sector, UINT count)
{
  DRESULT res = RES_ERROR;
  uint32_t timer;
  osEvent event;

  if (SD_CheckStatusWithTimeout(SD_TIMEOUT) < 0)
    return res;

  if (BSP_SD_WriteBlocks_DMA((uint32_t*)buff, (uint32_t)(sector), count) == MSD_OK)
  {
    event = osMessageGet(SDQueueID, SD_TIMEOUT);
    if (event.status == osEventMessage)
    {
      if (event.value.v == WRITE_CPLT_MSG)
      {
        timer = osKernelSysTick();
        while (osKernelSysTick() - timer < SD_TIMEOUT)
        {
          if (BSP_SD_GetCardState() == SD_TRANSFER_OK)
          {
            res = RES_OK;
            break;
          }
        }
      }
    }
  }

  return res;
}

DRESULT SD_ioctl(BYTE lun, BYTE cmd, void *buff)
{
  DRESULT res = RES_ERROR;
  BSP_SD_CardInfo CardInfo;

  if (Stat & STA_NOINIT) return RES_NOTRDY;

  switch (cmd)
  {
  case CTRL_SYNC:
    res = RES_OK;
    break;

  case GET_SECTOR_COUNT:
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockNbr;
    res = RES_OK;
    break;

  case GET_SECTOR_SIZE:
    BSP_SD_GetCardInfo(&CardInfo);
    *(WORD*)buff = CardInfo.LogBlockSize;
    res = RES_OK;
    break;

  case GET_BLOCK_SIZE:
    BSP_SD_GetCardInfo(&CardInfo);
    *(DWORD*)buff = CardInfo.LogBlockSize / SD_DEFAULT_BLOCK_SIZE;
    res = RES_OK;
    break;

  default:
    res = RES_PARERR;
  }

  return res;
}

/* DMA completion callbacks — post messages to the RTOS queue */
void BSP_SD_WriteCpltCallback(void)
{
  osMessagePut(SDQueueID, WRITE_CPLT_MSG, 0);
}

void BSP_SD_ReadCpltCallback(void)
{
  osMessagePut(SDQueueID, READ_CPLT_MSG, 0);
}
