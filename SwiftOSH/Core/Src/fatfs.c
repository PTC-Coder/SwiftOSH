/**
  ******************************************************************************
  * @file   fatfs.c — SwiftOSH
  * @brief  FatFS application layer, ported from SwiftOne
  ******************************************************************************
  */
#include "fatfs.h"
#include "RTC_Swift.h"

void MX_FATFS_Init(void)
{
  /* FatFS: Link the SD driver — done in AudioFiles_MountSDCard() */
}

/**
  * @brief  Return current time packed into a DWORD for FatFS timestamps.
  *         Called by FatFS internally when creating/modifying files.
  */
DWORD get_fattime(void)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  extern RTC_HandleTypeDef hrtc;

  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

  return (DWORD)(((DWORD)(sDate.Year + 2000 - 1980) << 25)
    | ((DWORD)sDate.Month << 21)
    | ((DWORD)sDate.Date << 16)
    | ((DWORD)sTime.Hours << 11)
    | ((DWORD)sTime.Minutes << 5)
    | ((DWORD)(sTime.Seconds / 2)));
}
