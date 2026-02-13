/**
  ******************************************************************************
  * @file    usb_device.h
  * @brief   USB Device init header — SwiftOSH
  ******************************************************************************
  */
#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx.h"
#include "stm32u5xx_hal.h"
#include "usbd_def.h"

void MX_USB_DEVICE_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_DEVICE__H__ */
