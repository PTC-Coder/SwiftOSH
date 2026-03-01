/**
  ******************************************************************************
  * @file    usbd_desc.c
  * @brief   USB Device descriptors — SwiftOSH
  ******************************************************************************
  */
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

#define USBD_VID                    0x0483
#define USBD_LANGID_STRING          1033
#define USBD_MANUFACTURER_STRING    "LabOfOrnithologyCCB"
#define USBD_PID_FS                 0x5751  /* ST Custom HID */
#define USBD_PRODUCT_STRING_FS      "SwiftOne Recorder"
#define USBD_CONFIGURATION_STRING_FS "Custom HID Config"
#define USBD_INTERFACE_STRING_FS    "Custom HID Interface"

static void Get_SerialNum(void);
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len);

uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef FS_Desc = {
  USBD_FS_DeviceDescriptor,
  USBD_FS_LangIDStrDescriptor,
  USBD_FS_ManufacturerStrDescriptor,
  USBD_FS_ProductStrDescriptor,
  USBD_FS_SerialStrDescriptor,
  USBD_FS_ConfigStrDescriptor,
  USBD_FS_InterfaceStrDescriptor,
};

__ALIGN_BEGIN uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
  0x12,                       /* bLength */
  USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
  0x00, 0x02,                 /* bcdUSB 2.00 */
  0x00,                       /* bDeviceClass */
  0x00,                       /* bDeviceSubClass */
  0x00,                       /* bDeviceProtocol */
  USB_MAX_EP0_SIZE,           /* bMaxPacketSize */
  LOBYTE(USBD_VID), HIBYTE(USBD_VID),
  LOBYTE(USBD_PID_FS), HIBYTE(USBD_PID_FS),
  0x00, 0x02,                 /* bcdDevice rel. 2.00 */
  USBD_IDX_MFC_STR,
  USBD_IDX_PRODUCT_STR,
  USBD_IDX_SERIAL_STR,
  USBD_MAX_NUM_CONFIGURATION
};

__ALIGN_BEGIN uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
  USB_LEN_LANGID_STR_DESC,
  USB_DESC_TYPE_STRING,
  LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING)
};

__ALIGN_BEGIN uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

__ALIGN_BEGIN uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
  USB_SIZ_STRING_SERIAL,
  USB_DESC_TYPE_STRING,
};

uint8_t *USBD_FS_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_FS_DeviceDesc);
  return USBD_FS_DeviceDesc;
}

uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = sizeof(USBD_LangIDDesc);
  return USBD_LangIDDesc;
}

uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
  return USBD_StrDesc;
}

uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  *length = USB_SIZ_STRING_SERIAL;
  Get_SerialNum();
  return (uint8_t *)USBD_StringSerial;
}

uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
  UNUSED(speed);
  USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
  return USBD_StrDesc;
}

static void Get_SerialNum(void)
{
  uint32_t d0 = *(uint32_t *)DEVICE_ID1;
  uint32_t d1 = *(uint32_t *)DEVICE_ID2;
  uint32_t d2 = *(uint32_t *)DEVICE_ID3;
  d0 += d2;
  if (d0 != 0)
  {
    IntToUnicode(d0, &USBD_StringSerial[2], 8);
    IntToUnicode(d1, &USBD_StringSerial[18], 4);
  }
}

static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
  for (uint8_t idx = 0; idx < len; idx++)
  {
    if ((value >> 28) < 0xA)
      pbuf[2 * idx] = (value >> 28) + '0';
    else
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    value <<= 4;
    pbuf[2 * idx + 1] = 0;
  }
}
