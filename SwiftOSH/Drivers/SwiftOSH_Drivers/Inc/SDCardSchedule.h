/* SDCardSchedule.h — Load plain text .sch schedule file from SD card */
#ifndef __SDCARDSCHEDULE_H
#define __SDCARDSCHEDULE_H

#include <stdint.h>

/**
  * @brief  Scan SD root for any .sch file, parse plain text schedule,
  *         write schedule blocks + day_skip to flash via read-modify-write.
  *         Renames to .done on success, .err on failure.
  * @retval 0 = no file found or success, 1 = error
  */
uint8_t SDSchedule_CheckAndApply(void);

#endif
