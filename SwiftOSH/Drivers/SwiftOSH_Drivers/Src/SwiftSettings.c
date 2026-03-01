/**
  ******************************************************************************
  * @file    SwiftSettings.c
  * @brief   Settings reader from flash — SwiftOSH (STM32U545RET6Q)
  *          Ported from SwiftOne SwiftSettings.c.
  *          Reads configuration stored at SETTINGS_BASE_ADDRESS (0x0807E000).
  ******************************************************************************
  */
#include "stm32u5xx_hal.h"
#include "SwiftSettings.h"
#include "GeneralDefines.h"

uint8_t SwiftSettings_GetCodecVariables(Codec_Config *cfg)
{
  uint32_t addr = SETTINGS_BASE_ADDRESS + CODEC_SETTINGS_OFFSET + 3;

  cfg->Codec_GAIN        = *(__IO uint8_t *)(addr++);
  cfg->Codec_PLL_PR      = *(__IO uint8_t *)(addr++);
  cfg->Codec_PLL_J       = *(__IO uint8_t *)(addr++);
  cfg->Codec_PLL_D_MSB   = *(__IO uint8_t *)(addr++);
  cfg->Codec_PLL_D_LSB   = *(__IO uint8_t *)(addr++);
  cfg->Codec_NADC        = *(__IO uint8_t *)(addr++);
  cfg->Codec_MADC        = *(__IO uint8_t *)(addr++);
  cfg->Codec_AOSR        = *(__IO uint8_t *)(addr++);
  cfg->Codec_BDIV        = *(__IO uint8_t *)(addr++);
  cfg->Codec_PRB         = *(__IO uint8_t *)(addr++);
  cfg->Codec_MIC_BIAS_CFG = *(__IO uint8_t *)(addr);

  return 1;
}

uint8_t SwiftSettings_GetWAVFileAttributes(WAVFile_Attributes *wav)
{
  uint32_t addr = SETTINGS_BASE_ADDRESS + WAVFILE_ATTRIBUTES_OFFSET + 2;
  uint8_t t1, t2, t3, t4;

  t1 = *(__IO uint8_t *)(addr++);
  t2 = *(__IO uint8_t *)(addr++);
  t3 = *(__IO uint8_t *)(addr++);
  t4 = *(__IO uint8_t *)(addr++);
  wav->FileSize = (((t4 << 24) | (t3 << 16) | (t2 << 8) | t1) / 2);

  wav->FilenameLength = *(__IO uint8_t *)(addr++);
  if (wav->FilenameLength > 20) wav->FilenameLength = 5;  /* Sanity: unprogrammed flash = 0xFF */
  for (int i = 0; i < wav->FilenameLength; i++)
    wav->Filename[i] = *(__IO uint8_t *)(addr++);
  wav->Filename[wav->FilenameLength] = 0;

  wav->SampleRate = *(__IO uint8_t *)((uint32_t)(SETTINGS_BASE_ADDRESS + CODEC_SETTINGS_OFFSET + 2)) * 1000;
  if (wav->SampleRate == 0 || wav->SampleRate > 96000)
    wav->SampleRate = 32000;  /* Default if unprogrammed */
  
  /* DEBUG: Force 32kHz until flash settings are corrected */
  wav->SampleRate = 32000;

  return 1;
}

uint8_t SwiftSettings_GetSwiftSchedule(Swift_Schedule *sched)
{
  uint32_t startAddr = SETTINGS_BASE_ADDRESS + SCHEDULE_STARTTIMES_OFFSET + 6;
  uint32_t stopAddr  = SETTINGS_BASE_ADDRESS + SCHEDULE_STOPTIMES_OFFSET + 7;
  int i = 0;

  sched->CurrentRecordNumber = 0;

  uint8_t flags = *(__IO uint8_t *)(SETTINGS_BASE_ADDRESS + SCHEDULE_STARTTIMES_OFFSET + 5);
  sched->StartsAtMidnightFlag = flags & 0x01;
  sched->EndsAtMidnightFlag   = (flags >> 1) & 0x01;
  sched->ScheduleType         = (flags >> 4) & 0x03;

  sched->StartDate.Year  = *(__IO uint8_t *)(SETTINGS_BASE_ADDRESS + SCHEDULE_STARTTIMES_OFFSET + 2);
  sched->StartDate.Month = *(__IO uint8_t *)(SETTINGS_BASE_ADDRESS + SCHEDULE_STARTTIMES_OFFSET + 3);
  sched->StartDate.Date  = *(__IO uint8_t *)(SETTINGS_BASE_ADDRESS + SCHEDULE_STARTTIMES_OFFSET + 4);

  sched->StopDate.Year  = *(__IO uint8_t *)(SETTINGS_BASE_ADDRESS + SCHEDULE_STOPTIMES_OFFSET + 2);
  sched->StopDate.Month = *(__IO uint8_t *)(SETTINGS_BASE_ADDRESS + SCHEDULE_STOPTIMES_OFFSET + 3);
  sched->StopDate.Date  = *(__IO uint8_t *)(SETTINGS_BASE_ADDRESS + SCHEDULE_STOPTIMES_OFFSET + 4);

  sched->NumberOfRecords = *(__IO uint8_t *)(startAddr++);
  if (sched->NumberOfRecords > 20) sched->NumberOfRecords = 0;  /* Sanity: unprogrammed flash = 0xFF */

  if (sched->ScheduleType == SCHEDULE_TYPE_ARBITRARY ||
      sched->ScheduleType == SCHEDULE_TYPE_CONTINUOUS)
  {
    while (i < sched->NumberOfRecords)
    {
      sched->StartStopTimes[i].RecordingStartTime.Hours   = *(__IO uint8_t *)(startAddr++);
      sched->StartStopTimes[i].RecordingStopTime.Hours    = *(__IO uint8_t *)(stopAddr++);
      sched->StartStopTimes[i].RecordingStartTime.Minutes = *(__IO uint8_t *)(startAddr++);
      sched->StartStopTimes[i].RecordingStopTime.Minutes  = *(__IO uint8_t *)(stopAddr++);
      i++;
    }
  }
  else if (sched->ScheduleType == SCHEDULE_TYPE_DUTYCYCLED)
  {
    sched->StartStopTimes[0].RecordingStartTime.Hours   = *(__IO uint8_t *)(startAddr);
    sched->StartStopTimes[0].RecordingStopTime.Hours    = *(__IO uint8_t *)(stopAddr);
    sched->StartStopTimes[0].RecordingStartTime.Minutes = *(__IO uint8_t *)(startAddr + 1);
    sched->StartStopTimes[0].RecordingStopTime.Minutes  = *(__IO uint8_t *)(stopAddr + 1);
    i = 1;
  }

  /* Zero remaining slots */
  while (i < 20)
  {
    sched->StartStopTimes[i].RecordingStartTime.Hours   = 0;
    sched->StartStopTimes[i].RecordingStopTime.Hours    = 0;
    sched->StartStopTimes[i].RecordingStartTime.Minutes = 0;
    sched->StartStopTimes[i].RecordingStopTime.Minutes  = 0;
    i++;
  }

  return 1;
}

uint8_t SwiftSettings_GetSTM32VersionInfo(uint8_t *DeviceID)
{
  uint32_t temp;

  temp = HAL_GetDEVID();
  DeviceID[0] = (uint8_t)(temp & 0xFF);
  DeviceID[1] = (uint8_t)((temp >> 8) & 0xFF);
  DeviceID[2] = (uint8_t)((temp >> 16) & 0xFF);
  DeviceID[3] = (uint8_t)((temp >> 24) & 0xFF);

  temp = HAL_GetREVID();
  DeviceID[4] = (uint8_t)(temp & 0xFF);
  DeviceID[5] = (uint8_t)((temp >> 8) & 0xFF);
  DeviceID[6] = (uint8_t)((temp >> 16) & 0xFF);
  DeviceID[7] = (uint8_t)((temp >> 24) & 0xFF);

  temp = HAL_GetHalVersion();
  DeviceID[8]  = (uint8_t)(temp & 0xFF);
  DeviceID[9]  = (uint8_t)((temp >> 8) & 0xFF);
  DeviceID[10] = (uint8_t)((temp >> 16) & 0xFF);
  DeviceID[11] = (uint8_t)((temp >> 24) & 0xFF);

  DeviceID[12] = __SWIFT_FIRMWARE_VERSION_RC;
  DeviceID[13] = __SWIFT_FIRMWARE_VERSION_SUB2;
  DeviceID[14] = __SWIFT_FIRMWARE_VERSION_SUB1;
  DeviceID[15] = __SWIFT_FIRMWARE_VERSION_MAIN;

  return 16;
}

uint8_t SwiftSettings_GetUniqueID(uint8_t *UniqueID)
{
  /* STM32U545 unique ID base: 0x0BFA0700 */
  uint32_t uid_base = 0x0BFA0700;

  UniqueID[0] = 7;   /* Report ID */
  UniqueID[1] = 12;  /* Array Size */

  for (int i = 0; i < 12; i++)
    UniqueID[2 + i] = *(__IO uint8_t *)(uid_base + i);

  return 14;
}

uint8_t SwiftSettings_GetDSTSettings(DST_Settings *dst)
{
  uint32_t addr = SETTINGS_BASE_ADDRESS + DST_OFFSET + 2;
  uint8_t t1, t2;

  t1 = *(__IO uint8_t *)(addr++);
  t2 = *(__IO uint8_t *)(addr++);
  dst->DST_Start_DOY = (t2 << 8) | t1;

  t1 = *(__IO uint8_t *)(addr++);
  t2 = *(__IO uint8_t *)(addr++);
  dst->DST_Stop_DOY = (t2 << 8) | t1;

  dst->DST_Offset_Hours   = *(__IO uint8_t *)(addr++);
  dst->DST_Offset_Minutes = *(__IO uint8_t *)(addr++);

  /* Inactive suffix */
  int i = 0;
  dst->DST_Filename_Suffix_Inactive[i] = *(__IO uint8_t *)(addr++);
  while (dst->DST_Filename_Suffix_Inactive[i] != 0 && i < 10)
  {
    i++;
    dst->DST_Filename_Suffix_Inactive[i] = *(__IO uint8_t *)(addr++);
  }

  /* Active suffix at offset +19 */
  addr = SETTINGS_BASE_ADDRESS + DST_OFFSET + 19;
  i = 0;
  dst->DST_Filename_Suffix_Active[i] = *(__IO uint8_t *)(addr++);
  while (dst->DST_Filename_Suffix_Active[i] != 0 && i < 10)
  {
    i++;
    dst->DST_Filename_Suffix_Active[i] = *(__IO uint8_t *)(addr++);
  }

  addr = SETTINGS_BASE_ADDRESS + DST_OFFSET + 31;
  dst->add_or_subtract_time = *(__IO uint8_t *)(addr);

  /* Read DST active flag from RTC backup register */
  extern RTC_HandleTypeDef hrtc;
  dst->DST_Active_Flag = (uint8_t)HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
  dst->Its_2AM_Flag = 0;

  return 1;
}

uint8_t SwiftSettings_GetConfigTextFromFlash(uint8_t *text, uint8_t packetNum)
{
  for (int i = 0; i < 48; i++)
    text[i] = *(__IO uint8_t *)((SETTINGS_BASE_ADDRESS + CONFIG_TEXTFILE_OFFSET + (packetNum * 48)) + i);

  return 1;
}
