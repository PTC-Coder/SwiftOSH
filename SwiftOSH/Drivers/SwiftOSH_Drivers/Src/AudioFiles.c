/**
  ******************************************************************************
  * @file    AudioFiles.c — SwiftOSH (STM32U545RET6Q)
  * @brief   SD card file/directory management for audio recording.
  *          Ported from SwiftOne AudioFiles.c.
  ******************************************************************************
  */
#include "stm32u5xx_hal.h"
#include "ff.h"
#include "ffconf.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"
#include "AudioFiles.h"
#include "GeneralDefines.h"
#include "RTC_Swift.h"
#include <string.h>
#include <stdio.h>

static char SD_Path[4];
static FATFS SDDISKFatFs;
static FILINFO FileInfo;
static FIL TextFile;

uint8_t AudioFiles_MountSDCard(SWIFT_ERRORS *SwiftError)
{
  uint8_t retSD;

  SwiftError->CallingFunction = FUNC_AudioFiles_MountSDCard;
  SwiftError->FatFsResult = FR_OK;

  retSD = FATFS_LinkDriver(&SD_Driver, SD_Path);
  if (retSD != 0)
  {
    SwiftError->ErrorNumber = 1;
    return 1;
  }

  SwiftError->FatFsResult = f_mount(&SDDISKFatFs, (TCHAR const*)SD_Path, 1);
  if (SwiftError->FatFsResult != FR_OK)
  {
    SwiftError->ErrorNumber = 2;
    return 1;
  }

  return 0;
}

uint8_t AudioFiles_NewRecordingDirectory(WAVFile_Attributes *RecordingDirectory)
{
  int DirectoryIndex = 0;
  char DirectoryName[13];
  char strDirectoryIndex[4];
  FRESULT ReturnResult;

  f_chdir("0:/");

  if (RecordingDirectory->RecordingDirFlag)
  {
    ReturnResult = f_chdir((char const*)RecordingDirectory->RecordingDirName);
    (void)ReturnResult;
  }
  else
  {
    while (DirectoryIndex < 255)
    {
      strcpy(DirectoryName, (char*)RecordingDirectory->Filename);

      if (DirectoryIndex < 10)
        strcat(DirectoryName, "00");
      else if (DirectoryIndex < 100)
        strcat(DirectoryName, "0");

      sprintf(strDirectoryIndex, "%d", DirectoryIndex);
      strcat(DirectoryName, strDirectoryIndex);

      ReturnResult = f_stat(DirectoryName, &FileInfo);
      if (ReturnResult == FR_NO_FILE)
        break;

      DirectoryIndex++;
    }

    if (DirectoryIndex == 255)
      return 1;

    ReturnResult = f_mkdir(DirectoryName);
    if (ReturnResult != FR_OK)
      return 1;

    ReturnResult = f_chdir(DirectoryName);
    if (ReturnResult != FR_OK)
      return 1;

    RecordingDirectory->RecordingDirFlag = 1;
    RecordingDirectory->RecordingDirName[0] = '/';
    RecordingDirectory->RecordingDirName[1] = '\0';
    strcat((char*)RecordingDirectory->RecordingDirName, DirectoryName);

    AudioFiles_NewConfigTextFile();
  }

  return 0;
}

uint8_t AudioFiles_NewDayDirectory(uint8_t *CustomLabel)
{
  char DirectoryName[20];
  char temp[11];
  FRESULT ReturnResult;

  strcpy(DirectoryName, (char*)CustomLabel);
  RTC_ReturnDateString(temp);
  strcat(DirectoryName, temp);

  ReturnResult = f_stat(DirectoryName, &FileInfo);
  if (ReturnResult == FR_NO_FILE)
  {
    ReturnResult = f_mkdir(DirectoryName);
    if (ReturnResult != FR_OK)
      return 1;
  }

  ReturnResult = f_chdir(DirectoryName);
  if (ReturnResult != FR_OK)
    return 1;

  return 0;
}

uint8_t AudioFiles_ChangeDirectory(void)
{
  f_chdir("0:/");
  return 1;
}

uint8_t AudioFiles_NewAudioFile(uint8_t *CustomLabel, uint8_t *utc_suffix, FIL *MyFile)
{
  FRESULT ReturnResult;
  char DateTimeString[16];
  char NewFileName[37];

  strcpy(NewFileName, (char*)CustomLabel);
  RTC_ReturnDateTimeFileName(DateTimeString);
  strcat(NewFileName, DateTimeString);

  if (!isClockWorking())
  {
    strcat(NewFileName, "extended");
  }

  if (utc_suffix != NULL)
    strcat(NewFileName, (char*)utc_suffix);

  strcat(NewFileName, ".wav");

  ReturnResult = f_open(MyFile, NewFileName, FA_CREATE_ALWAYS | FA_WRITE);
  if (ReturnResult != FR_OK)
    return 1;

  /* Make room for WAV header (46 bytes) */
  ReturnResult = f_lseek(MyFile, 46);
  if (ReturnResult != FR_OK)
    return 1;

  return 0;
}

uint8_t AudioFiles_GetDiskInfoString(char *DiskInfoString)
{
  FATFS *fs;
  DWORD fre_clust, fre_sect, tot_sect;
  uint8_t ReturnVal;

  f_getfree((TCHAR const*)SD_Path, &fre_clust, &fs);

  tot_sect = (fs->n_fatent - 2) * fs->csize;
  fre_sect = fre_clust * fs->csize;

  if (SDDISKFatFs.fs_type == FS_FAT32)
    ReturnVal = sprintf(DiskInfoString, "Drive filesystem: FAT32\r\nTotal drive space: %lu KB\r\nAvailable drive space: %lu KB\r\n\r\n", tot_sect / 2, fre_sect / 2);
  else if (SDDISKFatFs.fs_type == FS_EXFAT)
    ReturnVal = sprintf(DiskInfoString, "Drive filesystem: exFAT\r\nTotal drive space: %lu KB\r\nAvailable drive space: %lu KB\r\n\r\n", tot_sect / 2, fre_sect / 2);
  else
    ReturnVal = sprintf(DiskInfoString, "Drive filesystem: Unknown\r\nTotal drive space: %lu KB\r\nAvailable drive space: %lu KB\r\n\r\n", tot_sect / 2, fre_sect / 2);

  return ReturnVal;
}

uint8_t AudioFiles_NewConfigTextFile(void)
{
  FRESULT ReturnResult;
  uint8_t ConfigurationText[48];
  UINT bw;
  uint8_t i = 0;

  ReturnResult = f_open(&TextFile, "SwiftConfig.txt", FA_CREATE_ALWAYS | FA_WRITE);

  do {
    SwiftSettings_GetConfigTextFromFlash(ConfigurationText, i);
    f_write(&TextFile, &ConfigurationText[4], ConfigurationText[3], &bw);
    i++;
  } while (i <= ConfigurationText[2]);

  f_close(&TextFile);

  if (ReturnResult != FR_OK)
    return 1;

  return 0;
}

uint8_t AudioFiles_WriteHeader(SWIFT_ERRORS *SwiftError, FIL *MyFile)
{
  UINT bw;
  FSIZE_t tmpFileEndPointer;
  uint32_t SampleRateWord;

  BYTE WAVFileHeader[46] = {
    0x52, 0x49, 0x46, 0x46, /* "RIFF" */
    0x00, 0x00, 0x00, 0x00, /* File size - 8 */
    0x57, 0x41, 0x56, 0x45, /* "WAVE" */
    0x66, 0x6D, 0x74, 0x20, /* "fmt " */
    0x12, 0x00, 0x00, 0x00, /* Format data length (18) */
    0x01, 0x00, 0x01, 0x00, /* PCM, 1 channel */
    0x40, 0x1F, 0x00, 0x00, /* Sample rate (default 8000) */
    0x80, 0x3E, 0x00, 0x00, /* Byte rate */
    0x02, 0x00, 0x10, 0x00, /* Block align=2, Bits/sample=16 */
    0x00, 0x00,             /* Extra format bytes */
    0x64, 0x61, 0x74, 0x61, /* "data" */
    0x00, 0x20, 0x04, 0x00  /* Data size */
  };

  SwiftError->CallingFunction = FUNC_AudioFiles_WriteHeader;

  /* Read sample rate from flash settings (byte at offset 2) */
  uint8_t SampleRateByte = *(__IO uint8_t*)((uint32_t)(SETTINGS_BASE_ADDRESS + WAVFILE_ATTRIBUTES_OFFSET + 2));
  SampleRateWord = (uint32_t)SampleRateByte * 1000;

  /* File size fields */
  WAVFileHeader[4]  = (BYTE)((MyFile->obj.objsize - 8) & 0xFF);
  WAVFileHeader[5]  = (BYTE)(((MyFile->obj.objsize - 8) >> 8) & 0xFF);
  WAVFileHeader[6]  = (BYTE)(((MyFile->obj.objsize - 8) >> 16) & 0xFF);
  WAVFileHeader[7]  = (BYTE)(((MyFile->obj.objsize - 8) >> 24) & 0xFF);

  /* Sample rate */
  WAVFileHeader[24] = (BYTE)(SampleRateWord & 0xFF);
  WAVFileHeader[25] = (BYTE)((SampleRateWord >> 8) & 0xFF);
  WAVFileHeader[26] = (BYTE)((SampleRateWord >> 16) & 0xFF);
  WAVFileHeader[27] = (BYTE)((SampleRateWord >> 24) & 0xFF);

  /* Byte rate = SampleRate * BitsPerSample * Channels / 8 */
  SampleRateWord = (SampleRateWord * 16) >> 3;
  WAVFileHeader[28] = (BYTE)(SampleRateWord & 0xFF);
  WAVFileHeader[29] = (BYTE)((SampleRateWord >> 8) & 0xFF);
  WAVFileHeader[30] = (BYTE)((SampleRateWord >> 16) & 0xFF);
  WAVFileHeader[31] = (BYTE)((SampleRateWord >> 24) & 0xFF);

  /* Data chunk size */
  WAVFileHeader[42] = (BYTE)((MyFile->obj.objsize - 46) & 0xFF);
  WAVFileHeader[43] = (BYTE)(((MyFile->obj.objsize - 46) >> 8) & 0xFF);
  WAVFileHeader[44] = (BYTE)(((MyFile->obj.objsize - 46) >> 16) & 0xFF);
  WAVFileHeader[45] = (BYTE)(((MyFile->obj.objsize - 46) >> 24) & 0xFF);

  tmpFileEndPointer = MyFile->obj.objsize;

  SwiftError->FatFsResult = f_lseek(MyFile, 0);
  if (SwiftError->FatFsResult != FR_OK)
  {
    SwiftError->ErrorNumber = 1;
    return 1;
  }

  SwiftError->FatFsResult = f_write(MyFile, WAVFileHeader, 46, &bw);
  if (SwiftError->FatFsResult != FR_OK)
  {
    SwiftError->ErrorNumber = 3;
    return 1;
  }

  SwiftError->FatFsResult = f_lseek(MyFile, tmpFileEndPointer);
  if (SwiftError->FatFsResult != FR_OK)
  {
    SwiftError->ErrorNumber = 1;
    return 1;
  }

  return 0;
}

uint8_t AudioFiles_CloseFile(SWIFT_ERRORS *SwiftError, FIL *MyFile)
{
  SwiftError->CallingFunction = FUNC_AudioFiles_CloseFile;

  SwiftError->FatFsResult = f_truncate(MyFile);
  if (SwiftError->FatFsResult != FR_OK)
  {
    SwiftError->ErrorNumber = 2;
    return 1;
  }

  SwiftError->FatFsResult = f_close(MyFile);
  if (SwiftError->FatFsResult != FR_OK)
  {
    SwiftError->ErrorNumber = 3;
    return 1;
  }

  return 0;
}
