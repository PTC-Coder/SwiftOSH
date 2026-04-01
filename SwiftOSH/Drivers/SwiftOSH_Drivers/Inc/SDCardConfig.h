/* SDCardConfig.h — Load binary .cfg settings file from SD card */
#ifndef __SDCARDCONFIG_H
#define __SDCARDCONFIG_H

#include <stdint.h>

/**
  * @brief  Scan SD root for any .cfg file, parse 248-byte binary settings,
  *         write to flash.  Renames to .done on success, .err on failure.
  * @retval 0 = no file found or success, 1 = error
  */
uint8_t SDConfig_CheckAndApply(void);

#endif
