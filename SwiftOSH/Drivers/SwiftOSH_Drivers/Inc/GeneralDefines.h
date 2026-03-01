/* GeneralDefines.h — SwiftOSH (STM32U545RET6Q) */
#ifndef __GeneralDefines_H
#define __GeneralDefines_H

/* TLV320ADC3120 I2C address (7-bit 0x4D, shifted for HAL) */
#define CODEC_I2C_ADDRESS       (0x4D << 1)

/* Operating states */
#define STANDBY    0
#define RECORDING  1
#define ERROR      2

/* Flash settings storage — STM32U545 has 512KB flash, single bank, 8KB pages.
   Use last 8KB page (page 63) at 0x0807E000 for settings.
   CRITICAL: STM32U5 FLASH_TYPEPROGRAM_QUADWORD requires 16-byte aligned addresses.
   All offsets MUST be multiples of 16 (0x10). */
#define SETTINGS_BASE_ADDRESS     ((uint32_t)0x0807E000)

#define CODEC_SETTINGS_OFFSET       ((uint32_t)0)    /* 24 bytes → pad to 32 → next at 32 */
#define STM32_CLOCKDIV_OFFSET       ((uint32_t)32)   /* 24 bytes → pad to 32 → next at 64 */
#define WAVFILE_ATTRIBUTES_OFFSET   ((uint32_t)64)   /* 24 bytes → pad to 32 → next at 96 */
#define SCHEDULE_STARTTIMES_OFFSET  ((uint32_t)96)   /* 48 bytes → next at 144 */
#define SCHEDULE_STOPTIMES_OFFSET   ((uint32_t)144)  /* 48 bytes → next at 192 */
#define LATLONG_OFFSET              ((uint32_t)192)  /* 48 bytes → next at 240 */
#define DST_OFFSET                  ((uint32_t)240)  /* 32 bytes → next at 272 */
#define CONFIG_TEXTFILE_OFFSET      ((uint32_t)272)  /* variable, each packet 48 bytes */

#define SCHEDULE_TYPE_ARBITRARY     ((uint8_t)0x01)
#define SCHEDULE_TYPE_DUTYCYCLED    ((uint8_t)0x02)
#define SCHEDULE_TYPE_CONTINUOUS    ((uint8_t)0x03)

#define SET_ALARMB_MIDNIGHT         ((uint8_t)0x00)
#define SET_ALARMB_2AM              ((uint8_t)0x01)

#endif /* __GeneralDefines_H */
