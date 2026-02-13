/**
  ******************************************************************************
  * @file    usbd_conf.c
  * @brief   USB Device BSP — SwiftOSH (STM32U545 USB DRD FS)
  *          The STM32U545 has a USB DRD (Device/Host) peripheral, not OTG.
  ******************************************************************************
  */
#include "stm32u5xx.h"
#include "stm32u5xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_customhid.h"

PCD_HandleTypeDef hpcd_USB_DRD_FS;
void Error_Handler(void);
extern void SystemClock_Config(void);

/* ---- PCD MSP ---- */

void HAL_PCD_MspInit(PCD_HandleTypeDef *pcdHandle)
{
  if (pcdHandle->Instance == USB_DRD_FS)
  {
    /* Enable PWR clock and VDDUSB - required before USB peripheral access */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWREx_EnableVddUSB();
    
    /* Enable USB peripheral clock */
    __HAL_RCC_USB_FS_CLK_ENABLE();
    
    /* Configure USB GPIO: PA11=DM, PA12=DP — AF10 for USB */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF10_USB;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_NVIC_SetPriority(USB_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_IRQn);
  }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *pcdHandle)
{
  if (pcdHandle->Instance == USB_DRD_FS)
  {
    __HAL_RCC_USB_FS_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

    if (__HAL_RCC_PWR_IS_CLK_DISABLED())
    {
      __HAL_RCC_PWR_CLK_ENABLE();
      HAL_PWREx_DisableVddUSB();
      __HAL_RCC_PWR_CLK_DISABLE();
    }
    else
    {
      HAL_PWREx_DisableVddUSB();
    }
    HAL_NVIC_DisableIRQ(USB_IRQn);
  }
}

/* ---- PCD Callbacks ---- */

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData, (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
  USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, USBD_SPEED_FULL);
  USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
  USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData);
}

/* ---- LL Driver Interface ---- */

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
  if (pdev->id == DEVICE_FS)
  {
    hpcd_USB_DRD_FS.pData = pdev;
    pdev->pData = &hpcd_USB_DRD_FS;

    hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
    hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
    hpcd_USB_DRD_FS.Init.phy_itface    = PCD_PHY_EMBEDDED;
    hpcd_USB_DRD_FS.Init.Sof_enable    = DISABLE;
    hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
    hpcd_USB_DRD_FS.Init.lpm_enable    = DISABLE;
    hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK)
      Error_Handler();

    /* PMA (Packet Memory Area) allocation for USB DRD */
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x00, PCD_SNG_BUF, 0x18);  /* EP0 OUT */
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x80, PCD_SNG_BUF, 0x58);  /* EP0 IN */
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x01, PCD_SNG_BUF, 0x98);  /* HID OUT */
    HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x81, PCD_SNG_BUF, 0xD8);  /* HID IN */
  }
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
  HAL_PCD_DeInit(pdev->pData);
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
  USB_DRD_TypeDef *USBx = hpcd->Instance;
  
  __HAL_LOCK(hpcd);
  __HAL_PCD_ENABLE(hpcd);
  
  /* Add delay before BCDR write - this seems to be required for stability */
  HAL_Delay(5);
  
  /* Enable D+ pull-up to signal device presence to host */
  USBx->BCDR |= USB_BCDR_DPPU;
  
  __HAL_UNLOCK(hpcd);
  
  return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
  return (HAL_PCD_Stop(pdev->pData) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                   uint8_t ep_type, uint16_t ep_mps)
{
  return (HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return (HAL_PCD_EP_Close(pdev->pData, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return (HAL_PCD_EP_Flush(pdev->pData, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return (HAL_PCD_EP_SetStall(pdev->pData, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return (HAL_PCD_EP_ClrStall(pdev->pData, ep_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *)pdev->pData;
  if ((ep_addr & 0x80) == 0x80)
    return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
  else
    return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
  return (HAL_PCD_SetAddress(pdev->pData, dev_addr) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                     uint8_t *pbuf, uint32_t size)
{
  return (HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr,
                                           uint8_t *pbuf, uint32_t size)
{
  return (HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size) == HAL_OK) ? USBD_OK : USBD_FAIL;
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount((PCD_HandleTypeDef *)pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

void *USBD_static_malloc(uint32_t size)
{
  static uint32_t mem[(sizeof(USBD_CUSTOM_HID_HandleTypeDef) / 4 + 1)];
  (void)size;
  return mem;
}

void USBD_static_free(void *p)
{
  (void)p;
}
