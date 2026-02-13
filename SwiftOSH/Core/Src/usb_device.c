/**
  ******************************************************************************
  * @file    usb_device.c
  * @brief   USB Device init — SwiftOSH (STM32U545 USB DRD FS)
  ******************************************************************************
  */
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_customhid.h"
#include "usbd_custom_hid_if.h"

USBD_HandleTypeDef hUsbDeviceFS;
extern USBD_DescriptorsTypeDef FS_Desc;

void MX_USB_DEVICE_Init(void)
{
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
    Error_Handler();
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CUSTOM_HID) != USBD_OK)
    Error_Handler();
  if (USBD_CUSTOM_HID_RegisterInterface(&hUsbDeviceFS, &USBD_CustomHID_fops_FS) != USBD_OK)
    Error_Handler();
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
    Error_Handler();
}
