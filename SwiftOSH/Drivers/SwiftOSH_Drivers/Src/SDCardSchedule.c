/**
  ******************************************************************************
  * @file    SDCardSchedule.c
  * @brief   Load plain text .sch schedule file from SD card at boot.
  *          Ported from SwiftOne SDCardSchedule.c.
  *
  *          File format: plain text, one key=value per line. # = comment.
  *            schedule=arbitrary|dutycycle|continuous
  *            start_date=YY/MM/DD   stop_date=YY/MM/DD
  *            day_skip=N
  *            slot=HH:MM-HH:MM     (up to 20 entries)
  *            on_minutes=N  off_minutes=N  cycles=N  (dutycycle)
  *
  *          Uses read-modify-write of the settings flash page to preserve
  *          all other settings (codec, WAV file, lat/long, DST, etc.).
  ******************************************************************************
  */
#include "SDCardSchedule.h"
#include "ff.h"
#include "GeneralDefines.h"
#include "WriteToFlash.h"
#include "FlashLogging.h"
#include <string.h>
#include <stdlib.h>

/* 8KB page buffer for read-modify-write */
static uint8_t page_buf[8192];

static uint8_t FindSchFile(char *path, uint8_t maxlen)
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
      if (ext[0] == '.' &&
          (ext[1] == 's' || ext[1] == 'S') &&
          (ext[2] == 'c' || ext[2] == 'C') &&
          (ext[3] == 'h' || ext[3] == 'H'))
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

static void RenameSchFile(const char *path, const char *newExt)
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

/* Parse BCD date from "YY/MM/DD" string */
static uint8_t ParseBCD(const char *s)
{
  uint8_t hi = (s[0] - '0') & 0x0F;
  uint8_t lo = (s[1] - '0') & 0x0F;
  return (hi << 4) | lo;
}

uint8_t SDSchedule_CheckAndApply(void)
{
  static FIL schFile;
  char filepath[64];
  char line[128];

  if (!FindSchFile(filepath, sizeof(filepath)))
    return 0;

  WriteFlashNextEntry("<SDSchedule> Found .sch file\r\n");

  if (f_open(&schFile, filepath, FA_READ) != FR_OK)
  {
    WriteFlashNextEntry("<SDSchedule> Error opening file\r\n");
    RenameSchFile(filepath, ".err");
    return 1;
  }

  /* Read current settings page into RAM for read-modify-write */
  ReadFromFlash(page_buf, 0, 8192);

  /* Pointers into the page buffer for schedule blocks */
  uint8_t *startBlock = &page_buf[SCHEDULE_STARTTIMES_OFFSET];
  uint8_t *stopBlock  = &page_buf[SCHEDULE_STOPTIMES_OFFSET];

  /* Initialize schedule blocks to 0xFF */
  memset(startBlock, 0xFF, 48);
  memset(stopBlock, 0xFF, 48);

  uint8_t schedType = 0;
  uint8_t slotCount = 0;
  uint8_t daySkip = 0;
  uint8_t startsAtMidnight = 0;
  uint8_t endsAtMidnight = 0;
  uint8_t onMin = 0, offMin = 0, cycles = 1;

  /* Parse the file line by line */
  while (f_gets(line, sizeof(line), &schFile) != NULL)
  {
    /* Strip trailing whitespace */
    char *p = line + strlen(line) - 1;
    while (p >= line && (*p == '\r' || *p == '\n' || *p == ' ')) *p-- = 0;

    if (line[0] == '#' || line[0] == 0) continue;  /* comment or blank */

    if (strncmp(line, "schedule=", 9) == 0)
    {
      if (strcmp(&line[9], "arbitrary") == 0) schedType = SCHEDULE_TYPE_ARBITRARY;
      else if (strcmp(&line[9], "dutycycle") == 0) schedType = SCHEDULE_TYPE_DUTYCYCLED;
      else if (strcmp(&line[9], "continuous") == 0) schedType = SCHEDULE_TYPE_CONTINUOUS;
    }
    else if (strncmp(line, "start_date=", 11) == 0 && strlen(line) >= 19)
    {
      startBlock[2] = ParseBCD(&line[11]);  /* Year */
      startBlock[3] = ParseBCD(&line[14]);  /* Month */
      startBlock[4] = ParseBCD(&line[17]);  /* Date */
      stopBlock[2] = startBlock[2];  /* Copy as default */
      stopBlock[3] = startBlock[3];
      stopBlock[4] = startBlock[4];
    }
    else if (strncmp(line, "stop_date=", 10) == 0 && strlen(line) >= 18)
    {
      stopBlock[2] = ParseBCD(&line[10]);
      stopBlock[3] = ParseBCD(&line[13]);
      stopBlock[4] = ParseBCD(&line[16]);
    }
    else if (strncmp(line, "day_skip=", 9) == 0)
    {
      daySkip = (uint8_t)atoi(&line[9]);
      if (daySkip > 25) daySkip = 25;
    }
    else if (strncmp(line, "slot=", 5) == 0 && slotCount < 20)
    {
      /* slot=HH:MM-HH:MM */
      uint8_t sh = (uint8_t)atoi(&line[5]);
      uint8_t sm = (uint8_t)atoi(&line[8]);
      uint8_t eh = (uint8_t)atoi(&line[11]);
      uint8_t em = (uint8_t)atoi(&line[14]);
      startBlock[7 + slotCount * 2] = sh;
      startBlock[8 + slotCount * 2] = sm;
      stopBlock[7 + slotCount * 2]  = eh;
      stopBlock[8 + slotCount * 2]  = em;
      if (slotCount == 0 && sh == 0 && sm == 0) startsAtMidnight = 1;
      if (eh == 23 && em == 59) endsAtMidnight = 1;
      slotCount++;
    }
    else if (strncmp(line, "on_minutes=", 11) == 0)
      onMin = (uint8_t)atoi(&line[11]);
    else if (strncmp(line, "off_minutes=", 12) == 0)
      offMin = (uint8_t)atoi(&line[12]);
    else if (strncmp(line, "cycles=", 7) == 0)
      cycles = (uint8_t)atoi(&line[7]);
  }
  f_close(&schFile);

  /* Build flags byte */
  if (schedType == SCHEDULE_TYPE_DUTYCYCLED) startsAtMidnight = 1;
  uint8_t flags = (schedType << 4) | (endsAtMidnight << 1) | startsAtMidnight;
  startBlock[5] = flags;

  /* Number of records */
  if (schedType == SCHEDULE_TYPE_DUTYCYCLED)
  {
    startBlock[6] = cycles;
    startBlock[7] = onMin;
    startBlock[8] = 0;
    stopBlock[7]  = offMin;
    stopBlock[8]  = 0;
  }
  else
  {
    startBlock[6] = slotCount;
  }

  /* Write day_skip to codec byte 14 */
  page_buf[CODEC_SETTINGS_OFFSET + 14] = daySkip;

  /* Erase and rewrite the entire settings page */
  EraseFlashSector();
  /* Write in 16-byte (quadword) chunks — 8192 / 16 = 512 quadwords */
  for (uint16_t i = 0; i < 8192; i += 16)
  {
    /* Skip writing chunks that are all 0xFF (erased) */
    uint8_t allFF = 1;
    for (uint8_t j = 0; j < 16; j++)
    {
      if (page_buf[i + j] != 0xFF) { allFF = 0; break; }
    }
    if (!allFF)
    {
      HAL_FLASH_Unlock();
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
                        SETTINGS_BASE_ADDRESS + i,
                        (uint32_t)&page_buf[i]);
      HAL_FLASH_Lock();
    }
  }

  WriteFlashNextEntry("<SDSchedule> Schedule applied OK\r\n");
  RenameSchFile(filepath, ".done");
  return 0;
}
