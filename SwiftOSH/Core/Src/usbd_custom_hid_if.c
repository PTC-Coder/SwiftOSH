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
#include <string.h>

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t *Buffer);

/* Deferred flash operation queue — circular buffer to handle rapid back-to-back reports */
#define FLASH_QUEUE_DEPTH  16

typedef struct {
  uint8_t  reportId;
  uint8_t  data[64];
} FlashQueueEntry_t;

static volatile uint8_t        g_QueueHead = 0;
static volatile uint8_t        g_QueueTail = 0;
static FlashQueueEntry_t       g_FlashQueue[FLASH_QUEUE_DEPTH];

/* Legacy globals kept for compatibility */
volatile uint8_t g_FlashPending = 0;
volatile uint8_t g_FlashComplete = 0;
volatile uint8_t g_FlashReportId = 0;
uint8_t g_FlashBuffer[64] __attribute__((aligned(16)));

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

static int8_t CUSTOM_HID_OutEvent_FS(uint8_t *Buffer)
{
  uint8_t reportId = Buffer[0];

  switch (reportId)
  {
    case 4:  /* RTC Date and Time — no flash, handle immediately */
      RTC_SetDateTime(Buffer);
      break;

    case 9:  /* Jump to bootloader — check magic sequence first, handle immediately */
      /* Host sends: [0x09][0x55][0xAA][0xFF][0x00][0x00][0x00][0x00]
         Buffer[0] = report ID (9), Buffer[1..3] = magic = 0x55 0xAA 0xFF */
      if (Buffer[1] == 0x55 && Buffer[2] == 0xAA && Buffer[3] == 0xFF)
      {
        /* Write magic to RTC backup register DR3 — survives reset, not power cycle.
           main() checks this before any init and jumps to system bootloader. */
        extern RTC_HandleTypeDef hrtc;
        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, 0xDEADBEEF);
        NVIC_SystemReset();
      }
      break;

    default:
      /* All flash writes are deferred to main loop via USB_HID_ProcessFlash()
         so OutEvent returns immediately and USB host doesn't time out */
      {
        uint8_t next = (g_QueueTail + 1) % FLASH_QUEUE_DEPTH;
        if (next != g_QueueHead)  /* drop if full — shouldn't happen with depth 16 */
        {
          g_FlashQueue[g_QueueTail].reportId = reportId;
          memcpy(g_FlashQueue[g_QueueTail].data, Buffer, 64);
          g_QueueTail = next;
        }
      }
      break;
  }
  return 0;
}

/**
  * @brief  Process deferred flash operations. Call from main loop.
  *         Handles one pending report per call — call repeatedly until done.
  */
void USB_HID_ProcessFlash(void)
{
  if (g_QueueHead == g_QueueTail)
    return;  /* queue empty */

  FlashQueueEntry_t *entry = &g_FlashQueue[g_QueueHead];
  uint8_t reportId = entry->reportId;
  g_QueueHead = (g_QueueHead + 1) % FLASH_QUEUE_DEPTH;

  /* Flash stores the full report buffer as-is: [reportID][arraySize][data...]
     GET_REPORT reads it back verbatim so the host read-back verification passes.
     SwiftSettings readers skip byte 0 (reportID) and byte 1 (arraySize) with +2/+3 offsets. */
  switch (reportId)
  {
    case 2:  /* Codec Settings — erase entire page first, then write */
      EraseFlashSector();
      WriteToFlash(entry->data, CODEC_SETTINGS_OFFSET, 3);
      break;

    case 12: /* Clock Dividers */
      WriteToFlash(entry->data, STM32_CLOCKDIV_OFFSET, 3);
      break;

    case 14: /* WAV File Attributes */
      WriteToFlash(entry->data, WAVFILE_ATTRIBUTES_OFFSET, 3);
      break;

    case 16: /* Schedule Start Times */
      WriteToFlash(entry->data, SCHEDULE_STARTTIMES_OFFSET, 6);
      break;

    case 18: /* Schedule Stop Times */
      WriteToFlash(entry->data, SCHEDULE_STOPTIMES_OFFSET, 6);
      break;

    case 19: /* Config Text File — byte[1] is packet number */
      WriteToFlash(entry->data, CONFIG_TEXTFILE_OFFSET + (entry->data[1] * 48), 6);
      break;

    case 22: /* Lat/Long */
      WriteToFlash(entry->data, LATLONG_OFFSET, 6);
      break;

    case 24: /* DST Settings — byte[30] is DST status flag */
      RTC_SetDSTActiveFlag(entry->data[30]);
      WriteToFlash(entry->data, DST_OFFSET, 4);
      break;

    default:
      break;
  }
}
