/**
  ******************************************************************************
  * @file    SDCardConfig.c
  * @brief   Load binary .cfg settings file from SD card at boot.
  *          Ported from SwiftOne SDCardConfig.c.
  *
  *          File format: 248 bytes, 7 sequential blocks.
  *          Each block: byte[0]=reportID, byte[1]=blockSize, byte[2+]=payload.
  *          Block sizes are doubleword-aligned (multiples of 8).
  *
  *          Blocks: Report IDs 2(codec,24B), 12(clock,24B), 14(audio,24B),
  *                  16(startTimes,48B), 18(stopTimes,48B), 22(location,48B),
  *                  24(DST,32B).  Total = 248 bytes.
  ******************************************************************************
  */
#include "SDCardConfig.h"
#include "ff.h"
#include "GeneralDefines.h"
#include "WriteToFlash.h"
#include "FlashLogging.h"
#include "RTC_Swift.h"
#include <string.h>

#define CFG_FILE_SIZE  248

/* Scan SD root for any .cfg file (case-insensitive) */
static uint8_t FindCfgFile(char *path, uint8_t maxlen)
{
  DIR dir;
  FILINFO fno;

  if (f_opendir(&dir, "0:/") != FR_OK) return 0;

  while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != 0)
  {
    uint8_t len = strlen(fno.fname);
    if (len > 4)
    {
      char *ext = &fno.fname[len - 4];
      if ((ext[0] == '.' ) &&
          (ext[1] == 'c' || ext[1] == 'C') &&
          (ext[2] == 'f' || ext[2] == 'F') &&
          (ext[3] == 'g' || ext[3] == 'G'))
      {
        snprintf(path, maxlen, "0:/%s", fno.fname);
        f_closedir(&dir);
        return 1;
      }
    }
  }
  f_closedir(&dir);
  return 0;
}

static void RenameCfgFile(const char *path, const char *newExt)
{
  char newpath[80];
  strncpy(newpath, path, sizeof(newpath) - 1);
  newpath[sizeof(newpath) - 1] = 0;
  uint8_t len = strlen(newpath);
  if (len > 4)
  {
    newpath[len - 4] = 0;
    strncat(newpath, newExt, sizeof(newpath) - strlen(newpath) - 1);
    f_rename(path, newpath);
  }
}

uint8_t SDConfig_CheckAndApply(void)
{
  static uint8_t cfg_buf[CFG_FILE_SIZE];
  static FIL cfgFile;
  char filepath[64];
  UINT bytes_read;

  if (!FindCfgFile(filepath, sizeof(filepath)))
    return 0;  /* No .cfg file found — normal, not an error */

  WriteFlashNextEntry("<SDConfig> Found .cfg file\r\n");

  if (f_open(&cfgFile, filepath, FA_READ) != FR_OK)
  {
    WriteFlashNextEntry("<SDConfig> Error opening file\r\n");
    RenameCfgFile(filepath, ".err");
    return 1;
  }

  if (f_read(&cfgFile, cfg_buf, CFG_FILE_SIZE, &bytes_read) != FR_OK ||
      bytes_read != CFG_FILE_SIZE)
  {
    WriteFlashNextEntry("<SDConfig> Bad file size\r\n");
    f_close(&cfgFile);
    RenameCfgFile(filepath, ".err");
    return 1;
  }
  f_close(&cfgFile);

  /* Erase settings page once, then write each block */
  EraseFlashSector();

  uint16_t offset = 0;
  while (offset < CFG_FILE_SIZE)
  {
    uint8_t reportId   = cfg_buf[offset];
    uint8_t blockSize  = cfg_buf[offset + 1];

    if (blockSize < 2 || (blockSize % 8) != 0 ||
        (offset + blockSize) > CFG_FILE_SIZE)
    {
      WriteFlashNextEntry("<SDConfig> Bad block at offset\r\n");
      RenameCfgFile(filepath, ".err");
      return 1;
    }

    /* Write block to flash using same offsets as USB HID handler */
    switch (reportId)
    {
      case 2:  WriteToFlash(&cfg_buf[offset], CODEC_SETTINGS_OFFSET, 3); break;
      case 12: WriteToFlash(&cfg_buf[offset], STM32_CLOCKDIV_OFFSET, 3); break;
      case 14: WriteToFlash(&cfg_buf[offset], WAVFILE_ATTRIBUTES_OFFSET, 3); break;
      case 16: WriteToFlash(&cfg_buf[offset], SCHEDULE_STARTTIMES_OFFSET, 6); break;
      case 18: WriteToFlash(&cfg_buf[offset], SCHEDULE_STOPTIMES_OFFSET, 6); break;
      case 22: WriteToFlash(&cfg_buf[offset], LATLONG_OFFSET, 6); break;
      case 24:
        RTC_SetDSTActiveFlag(cfg_buf[offset + 30]);
        WriteToFlash(&cfg_buf[offset], DST_OFFSET, 4);
        break;
      default:
        break;
    }
    offset += blockSize;
  }

  WriteFlashNextEntry("<SDConfig> Settings applied OK\r\n");
  RenameCfgFile(filepath, ".done");
  return 0;
}
