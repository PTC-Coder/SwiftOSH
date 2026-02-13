/**
  ******************************************************************************
  * @file    usbd_custom_hid_if.c
  * @brief   USB Custom HID interface — SwiftOSH
  *          Same HID report descriptor as SwiftOne (251 bytes)
  ******************************************************************************
  */
#include "usbd_custom_hid_if.h"
#include "main.h"
#include "GeneralDefines.h"
#include "WriteToFlash.h"
#include "RTC_Swift.h"

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/* Identical HID report descriptor from the original SwiftOne firmware */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  0x06, 0xFF, 0x00,       /* USAGE_PAGE (Vendor Defined) */
  0x09, 0x01,              /* USAGE (1) */
  0xA1, 0x01,              /* COLLECTION (Application) */

  /* Report ID 1/2: Codec Settings (24 bytes) */
  0x85, 0x01, 0x09, 0x01, 0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, 0x18, 0x81, 0x82,
  0x85, 0x02, 0x09, 0x01, 0x91, 0x82,

  /* Report ID 3/4: RTC Date/Time (13 bytes) */
  0x85, 0x03, 0x09, 0x02, 0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, 0x0D, 0x81, 0x82,
  0x85, 0x04, 0x09, 0x02, 0x91, 0x82,

  /* Report ID 6: FW Version (12 bytes) */
  0x85, 0x06, 0x09, 0x04, 0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, 0x0C, 0x81, 0x82,

  /* Report ID 7: Unique ID (14 bytes) */
  0x85, 0x07, 0x09, 0x05, 0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, 0x0E, 0x81, 0x82,

  /* Report ID 8: Battery Voltage (3 bytes) */
  0x85, 0x08, 0x09, 0x06, 0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, 0x03, 0x81, 0x82,

  /* Report ID 9: Bootloader (8 bytes OUT) */
  0x85, 0x09, 0x09, 0x07, 0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, 0x08, 0x91, 0x82,

  /* Report ID 11/12: Clock Dividers (24 bytes) */
  0x85, 0x0B, 0x09, 0x09, 0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, 0x18, 0x81, 0x82,
  0x85, 0x0C, 0x09, 0x09, 0x91, 0x82,

  /* Report ID 13/14: WAV File Attributes (24 bytes) */
  0x85, 0x0D, 0x09, 0x0A, 0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, 0x18, 0x81, 0x82,
  0x85, 0x0E, 0x09, 0x0A, 0x91, 0x82,

  /* Report ID 15/16: Schedule Start Times (48 bytes) */
  0x85, 0x0F, 0x09, 0x0B, 0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, 0x30, 0x81, 0x82,
  0x85, 0x10, 0x09, 0x0B, 0x91, 0x82,

  /* Report ID 17/18: Schedule Stop Times (48 bytes) */
  0x85, 0x11, 0x09, 0x0C, 0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, 0x30, 0x81, 0x82,
  0x85, 0x12, 0x09, 0x0C, 0x91, 0x82,

  /* Report ID 19: Config Text File (48 bytes OUT) */
  0x85, 0x13, 0x09, 0x0D, 0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, 0x30, 0x91, 0x82,

  /* Report ID 21/22: Lat/Long (48 bytes) */
  0x85, 0x15, 0x09, 0x0E, 0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, 0x30, 0x81, 0x82,
  0x85, 0x16, 0x09, 0x0E, 0x91, 0x82,

  /* Report ID 23/24: DST Settings (32 bytes) */
  0x85, 0x17, 0x09, 0x0F, 0x15, 0x00, 0x26, 0xFF, 0x00,
  0x75, 0x08, 0x95, 0x20, 0x81, 0x82,
  0x85, 0x18, 0x09, 0x0F, 0x91, 0x82,

  0xC0  /* END_COLLECTION */
};

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS = {
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

static int8_t CUSTOM_HID_Init_FS(void)
{
  return (USBD_OK);
}

static int8_t CUSTOM_HID_DeInit_FS(void)
{
  return (USBD_OK);
}

static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  /* Access the full report buffer from the USB device handle */
  extern USBD_HandleTypeDef hUsbDeviceFS;
  USBD_CUSTOM_HID_HandleTypeDef *hhid =
    (USBD_CUSTOM_HID_HandleTypeDef *)hUsbDeviceFS.pClassData;
  uint8_t *Buffer = hhid->Report_buf;

  switch (Buffer[0])
  {
    case 2:  /* Codec Settings */
      EraseFlashSector();
      WriteToFlash(Buffer, CODEC_SETTINGS_OFFSET, 3);
      break;

    case 4:  /* RTC Date and Time */
      RTC_SetDateTime(Buffer);
      break;

    case 9:  /* Jump to bootloader */
      HAL_RCC_DeInit();
      SysTick->CTRL = 0;
      SysTick->LOAD = 0;
      SysTick->VAL  = 0;
      __set_PRIMASK(1);
      NVIC_SystemReset();
      break;

    case 12: /* Clock Dividers */
      WriteToFlash(Buffer, STM32_CLOCKDIV_OFFSET, 3);
      break;

    case 14: /* WAV File Attributes */
      WriteToFlash(Buffer, WAVFILE_ATTRIBUTES_OFFSET, 3);
      break;

    case 16: /* Schedule Start Times */
      WriteToFlash(Buffer, SCHEDULE_STARTTIMES_OFFSET, 6);
      break;

    case 18: /* Schedule Stop Times */
      WriteToFlash(Buffer, SCHEDULE_STOPTIMES_OFFSET, 6);
      break;

    case 19: /* Config Text File */
      WriteToFlash(Buffer, CONFIG_TEXTFILE_OFFSET + (Buffer[1] * 48), 6);
      break;

    case 22: /* Lat/Long */
      WriteToFlash(Buffer, LATLONG_OFFSET, 6);
      break;

    case 24: /* DST Settings */
      RTC_SetDSTActiveFlag(Buffer[30]);
      WriteToFlash(Buffer, DST_OFFSET, 4);
      break;

    default:
      break;
  }
  return 0;
}
