/* SwiftSettings.h — SwiftOSH */
#ifndef __SwiftSettings_H
#define __SwiftSettings_H

#include "stm32u5xx_hal.h"

#define __SWIFT_FIRMWARE_VERSION_MAIN   (0x01)
#define __SWIFT_FIRMWARE_VERSION_SUB1   (0x00)
#define __SWIFT_FIRMWARE_VERSION_SUB2   (0x03)
#define __SWIFT_FIRMWARE_VERSION_RC     (0x14 )

typedef struct {
  uint8_t Codec_GAIN;
  uint8_t Codec_NADC;
  uint8_t Codec_MADC;
  uint8_t Codec_AOSR;
  uint8_t Codec_BDIV;
  uint8_t Codec_PRB;
  uint8_t Codec_PLL_J;
  uint8_t Codec_PLL_D_MSB;
  uint8_t Codec_PLL_D_LSB;
  uint8_t Codec_PLL_PR;
  uint8_t Codec_MIC_BIAS_CFG;
} Codec_Config;

typedef struct {
  uint16_t DST_Start_DOY;
  uint16_t DST_Stop_DOY;
  uint8_t  DST_Offset_Hours;
  uint8_t  DST_Offset_Minutes;
  uint8_t  DST_Filename_Suffix_Active[11];
  uint8_t  DST_Filename_Suffix_Inactive[11];
  uint8_t *DST_Suffix_To_Use;
  uint8_t  DST_Active_Flag;
  uint8_t  add_or_subtract_time;
  uint16_t Current_DOY;
  uint16_t TotalDaysInYear;
  uint8_t  Its_2AM_Flag;
} DST_Settings;

typedef struct {
  uint32_t FileSize;
  uint8_t  Filename[10];
  uint8_t  FilenameLength;
  uint8_t  IsFileOpen;
  uint8_t  DirectoryLevel;
  uint32_t SampleRate;
  uint8_t  RecordingDirFlag;
  uint8_t  RecordingDirName[14];
} WAVFile_Attributes;

typedef struct {
  RTC_TimeTypeDef RecordingStartTime;
  RTC_TimeTypeDef RecordingStopTime;
} Schedule_Times;

typedef struct {
  uint8_t         ScheduleType;
  uint8_t         StartsAtMidnightFlag;
  uint8_t         EndsAtMidnightFlag;
  uint8_t         NumberOfRecords;
  uint8_t         CurrentRecordNumber;
  RTC_DateTypeDef StartDate;
  RTC_DateTypeDef StopDate;
  Schedule_Times  StartStopTimes[20];
} Swift_Schedule;

uint8_t SwiftSettings_GetCodecVariables(Codec_Config *);
uint8_t SwiftSettings_GetWAVFileAttributes(WAVFile_Attributes *);
uint8_t SwiftSettings_GetSwiftSchedule(Swift_Schedule *);
uint8_t SwiftSettings_GetSTM32VersionInfo(uint8_t *);
uint8_t SwiftSettings_GetUniqueID(uint8_t *);
uint8_t SwiftSettings_GetDSTSettings(DST_Settings *);
uint8_t SwiftSettings_GetConfigTextFromFlash(uint8_t *, uint8_t);

#endif
