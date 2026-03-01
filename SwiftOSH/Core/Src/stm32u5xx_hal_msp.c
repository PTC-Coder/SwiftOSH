/**
  ******************************************************************************
  * @file    stm32u5xx_hal_msp.c
  * @brief   MSP Initialization — SwiftOSH (STM32U545RET6Q)
  ******************************************************************************
  */
#include "main.h"
#include <string.h>

extern DMA_HandleTypeDef hdma_sai1_a;
extern DMA_NodeTypeDef   SAI_DMA_Node1;
extern DMA_NodeTypeDef   SAI_DMA_Node2;
extern DMA_QListTypeDef  SAI_DMA_Queue;

/* Buffer size must match main.c */
#define BUFFER_SIZE       32768
#define BUFFER_SIZE_DIV2  (BUFFER_SIZE / 2)
extern uint8_t I2SRxBuffer[];

void HAL_MspInit(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
}

/* ---- ADC MSP ---- */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin  = GPIO_PIN_0;  /* PA0 = VBAT_SCALED */
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    __HAL_RCC_ADC12_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
  }
}

/* ---- I2C MSP ---- */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_6 | GPIO_PIN_7;  /* PB6=SCL, PB7=SDA */
    g.Mode      = GPIO_MODE_AF_OD;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &g);
    __HAL_RCC_I2C1_CLK_ENABLE();
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    __HAL_RCC_I2C1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
  }
}

/* ---- LPTIM MSP ---- */
void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef *hlptim)
{
  if (hlptim->Instance == LPTIM1)
  {
    __HAL_RCC_LPTIM1_CLK_ENABLE();
    HAL_NVIC_SetPriority(LPTIM1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(LPTIM1_IRQn);
  }
  else if (hlptim->Instance == LPTIM2)
  {
    /* LPTIM2 used for LED blink timing — no GPIO AF, just clock + IRQ */
    __HAL_RCC_LPTIM2_CLK_ENABLE();
    HAL_NVIC_SetPriority(LPTIM2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(LPTIM2_IRQn);
  }
}

void HAL_LPTIM_MspDeInit(LPTIM_HandleTypeDef *hlptim)
{
  if (hlptim->Instance == LPTIM1)
  {
    __HAL_RCC_LPTIM1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6);
    HAL_NVIC_DisableIRQ(LPTIM1_IRQn);
  }
  else if (hlptim->Instance == LPTIM2)
  {
    __HAL_RCC_LPTIM2_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(LPTIM2_IRQn);
  }
}

/* ---- RTC MSP ---- */
void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc_arg)
{
  if (hrtc_arg->Instance == RTC)
  {
    __HAL_RCC_RTC_ENABLE();
    __HAL_RCC_RTCAPB_CLK_ENABLE();
    HAL_NVIC_SetPriority(RTC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(RTC_IRQn);
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc_arg)
{
  if (hrtc_arg->Instance == RTC)
  {
    __HAL_RCC_RTC_DISABLE();
    HAL_NVIC_DisableIRQ(RTC_IRQn);
  }
}

/* ---- SD MSP ---- */
void HAL_SD_MspInit(SD_HandleTypeDef *hsd)
{
  if (hsd->Instance == SDMMC1)
  {
    __HAL_RCC_SDMMC1_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    /* PC8=D0, PC9=D1, PC10=D2, PC11=D3 — need pull-ups for SD protocol */
    g.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOC, &g);

    /* PC12=CLK — no pull-up on clock line */
    g.Pin  = GPIO_PIN_12;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &g);

    /* PD2=CMD — needs pull-up */
    g.Pin  = GPIO_PIN_2;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &g);

    HAL_NVIC_SetPriority(SDMMC1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
  }
}

void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd)
{
  if (hsd->Instance == SDMMC1)
  {
    __HAL_RCC_SDMMC1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
    HAL_NVIC_DisableIRQ(SDMMC1_IRQn);
  }
}

/* ---- SAI MSP ---- */
static uint32_t SAI1_client = 0;

void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI1_Block_A)
  {
    if (SAI1_client == 0)
    {
      __HAL_RCC_SAI1_CLK_ENABLE();
      HAL_NVIC_SetPriority(SAI1_IRQn, 5, 0);
      HAL_NVIC_EnableIRQ(SAI1_IRQn);
    }
    SAI1_client++;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    /* PA8=SAI1_SCK_A (BCLK), PA9=SAI1_FS_A (WCLK), PA10=SAI1_SD_A (DOUT) */
    g.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF13_SAI1;
    HAL_GPIO_Init(GPIOA, &g);

    /* PB8=SAI1_MCLK_A */
    g.Pin       = GPIO_PIN_8;
    g.Alternate = GPIO_AF13_SAI1;
    HAL_GPIO_Init(GPIOB, &g);

    /* GPDMA1 Channel 0 for SAI1_A RX — linked-list circular mode.
       STM32U5 GPDMA does not support DMA_CIRCULAR in Init.Mode.
       Circular transfers require a linked-list queue with circular mode set.
       
       IMPORTANT: GPDMA linked-list mode has NO half-transfer interrupt.
       We use TWO nodes, each transferring half the buffer:
         Node1 → first half  → TC fires → HAL calls RxHalfCplt callback
         Node2 → second half → TC fires → HAL calls RxCplt callback
       The queue is circular: Node2 links back to Node1. */
    __HAL_RCC_GPDMA1_CLK_ENABLE();

    /* Step 1: Init the DMA handle in linked-list mode */
    hdma_sai1_a.Instance                         = GPDMA1_Channel0;
    hdma_sai1_a.InitLinkedList.Priority          = DMA_HIGH_PRIORITY;
    hdma_sai1_a.InitLinkedList.LinkStepMode      = DMA_LSM_FULL_EXECUTION;
    hdma_sai1_a.InitLinkedList.LinkAllocatedPort = DMA_LINK_ALLOCATED_PORT1;
    hdma_sai1_a.InitLinkedList.TransferEventMode = DMA_TCEM_LAST_LL_ITEM_TRANSFER;
    hdma_sai1_a.InitLinkedList.LinkedListMode    = DMA_LINKEDLIST_CIRCULAR;
    if (HAL_DMAEx_List_Init(&hdma_sai1_a) != HAL_OK)
      Error_Handler();

    /* Step 2: Build TWO nodes — each transfers half the buffer.
       Zero the nodes first to clear any stale state from a previous session. */
    memset(&SAI_DMA_Node1, 0, sizeof(SAI_DMA_Node1));
    memset(&SAI_DMA_Node2, 0, sizeof(SAI_DMA_Node2));

    DMA_NodeConfTypeDef nodeConf = {0};
    nodeConf.NodeType                         = DMA_GPDMA_LINEAR_NODE;
    nodeConf.Init.Request                     = GPDMA1_REQUEST_SAI1_A;
    nodeConf.Init.BlkHWRequest                = DMA_BREQ_SINGLE_BURST;
    nodeConf.Init.Direction                   = DMA_PERIPH_TO_MEMORY;
    nodeConf.Init.SrcInc                      = DMA_SINC_FIXED;
    nodeConf.Init.DestInc                     = DMA_DINC_INCREMENTED;
    nodeConf.Init.SrcDataWidth                = DMA_SRC_DATAWIDTH_HALFWORD;
    nodeConf.Init.DestDataWidth               = DMA_DEST_DATAWIDTH_HALFWORD;
    nodeConf.Init.SrcBurstLength              = 1;
    nodeConf.Init.DestBurstLength             = 1;
    nodeConf.Init.TransferAllocatedPort       = DMA_SRC_ALLOCATED_PORT0 | DMA_DEST_ALLOCATED_PORT1;
    nodeConf.Init.TransferEventMode           = DMA_TCEM_EACH_LL_ITEM_TRANSFER;
    nodeConf.Init.Mode                        = DMA_NORMAL;

    /* Node 1: SAI1_A DR → first half of buffer (BUFFER_SIZE_DIV2 bytes = BUFFER_SIZE_DIV2/2 halfwords) */
    nodeConf.SrcAddress = (uint32_t)&SAI1_Block_A->DR;
    nodeConf.DstAddress = (uint32_t)&I2SRxBuffer[0];
    nodeConf.DataSize   = BUFFER_SIZE_DIV2;  /* bytes */
    if (HAL_DMAEx_List_BuildNode(&nodeConf, &SAI_DMA_Node1) != HAL_OK)
      Error_Handler();

    /* Node 2: SAI1_A DR → second half of buffer */
    nodeConf.DstAddress = (uint32_t)&I2SRxBuffer[BUFFER_SIZE_DIV2];
    nodeConf.DataSize   = BUFFER_SIZE_DIV2;  /* bytes */
    if (HAL_DMAEx_List_BuildNode(&nodeConf, &SAI_DMA_Node2) != HAL_OK)
      Error_Handler();

    /* Step 3: Insert both nodes into queue and make it circular.
       Reset queue first in case this is a re-init after Stop 2 wake-up. */
    HAL_DMAEx_List_ResetQ(&SAI_DMA_Queue);
    if (HAL_DMAEx_List_InsertNode_Tail(&SAI_DMA_Queue, &SAI_DMA_Node1) != HAL_OK)
      Error_Handler();
    if (HAL_DMAEx_List_InsertNode_Tail(&SAI_DMA_Queue, &SAI_DMA_Node2) != HAL_OK)
      Error_Handler();
    if (HAL_DMAEx_List_SetCircularMode(&SAI_DMA_Queue) != HAL_OK)
      Error_Handler();

    /* Step 4: Link the queue to the DMA handle */
    if (HAL_DMAEx_List_LinkQ(&hdma_sai1_a, &SAI_DMA_Queue) != HAL_OK)
      Error_Handler();

    __HAL_LINKDMA(hsai, hdmarx, hdma_sai1_a);
  }
}

void HAL_SAI_MspDeInit(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI1_Block_A)
  {
    SAI1_client--;
    if (SAI1_client == 0)
    {
      __HAL_RCC_SAI1_CLK_DISABLE();
      HAL_NVIC_DisableIRQ(SAI1_IRQn);
    }
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);
    HAL_DMAEx_List_DeInit(hsai->hdmarx);
    HAL_DMAEx_List_ResetQ(&SAI_DMA_Queue);
  }
}
