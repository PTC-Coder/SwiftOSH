/**
  ******************************************************************************
  * @file    usbd_custom_hid_if.h
  * @brief   Custom HID interface header — SwiftOSH
  ******************************************************************************
  */
#ifndef __USBD_CUSTOM_HID_IF_H__
#define __USBD_CUSTOM_HID_IF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_customhid.h"

extern USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS;

/* Call from main loop to process deferred flash operations */
void USB_HID_ProcessFlash(void);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CUSTOM_HID_IF_H__ */
