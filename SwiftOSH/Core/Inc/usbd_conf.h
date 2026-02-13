/**
  ******************************************************************************
  * @file    usbd_conf.h
  * @brief   USB Device configuration header — SwiftOSH
  ******************************************************************************
  */
#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "stm32u5xx.h"
#include "stm32u5xx_hal.h"

/* ---- USB configuration ---- */
#define USBD_MAX_NUM_INTERFACES        1U
#define USBD_MAX_NUM_CONFIGURATION     1U
#define USBD_MAX_STR_DESC_SIZ         512U
#define USBD_DEBUG_LEVEL               0U
#define USBD_LPM_ENABLED               0U
#define USBD_SELF_POWERED              1U

/* Custom HID specific */
#define USBD_CUSTOMHID_OUTREPORT_BUF_SIZE  49U
#define USBD_CUSTOM_HID_REPORT_DESC_SIZE  251U
#define CUSTOM_HID_FS_BINTERVAL           0x05U

#define DEVICE_FS  0

/* Memory management macros */
#define USBD_malloc   malloc
#define USBD_free     free
#define USBD_memset   memset
#define USBD_memcpy   memcpy
#define USBD_Delay    HAL_Delay

#if (USBD_DEBUG_LEVEL > 0)
#define USBD_UsrLog(...)  printf(__VA_ARGS__); printf("\n");
#else
#define USBD_UsrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 1)
#define USBD_ErrLog(...)  printf("ERROR: "); printf(__VA_ARGS__); printf("\n");
#else
#define USBD_ErrLog(...)
#endif

#if (USBD_DEBUG_LEVEL > 2)
#define USBD_DbgLog(...)  printf("DEBUG : "); printf(__VA_ARGS__); printf("\n");
#else
#define USBD_DbgLog(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF__H__ */
