/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
FDCAN_FilterTypeDef FDCAN1_RxFilter;
FDCAN_FilterTypeDef FDCAN2_RxFilter;

FDCAN_RxFrame_TypeDef FDCAN1_RxFrame;
FDCAN_RxFrame_TypeDef FDCAN2_RxFrame;

FDCAN_RxHeaderTypeDef fdcan1_RxHeader;
FDCAN_TxHeaderTypeDef fdcan1_TxHeader;

FDCAN_RxHeaderTypeDef fdcan2_RxHeader;
FDCAN_TxHeaderTypeDef fdcan2_TxHeader;

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 3;
  hfdcan1.Init.NominalSyncJumpWidth = 10;
  hfdcan1.Init.NominalTimeSeg1 = 29;
  hfdcan1.Init.NominalTimeSeg2 = 10;
  hfdcan1.Init.DataPrescaler = 3;
  hfdcan1.Init.DataSyncJumpWidth = 10;
  hfdcan1.Init.DataTimeSeg1 = 29;
  hfdcan1.Init.DataTimeSeg2 = 10;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 0x00000000;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 2;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 2;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}
/* FDCAN2 init function */
void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = ENABLE;
  hfdcan2.Init.NominalPrescaler = 3;
  hfdcan2.Init.NominalSyncJumpWidth = 10;
  hfdcan2.Init.NominalTimeSeg1 = 29;
  hfdcan2.Init.NominalTimeSeg2 = 10;
  hfdcan2.Init.DataPrescaler = 3;
  hfdcan2.Init.DataSyncJumpWidth = 10;
  hfdcan2.Init.DataTimeSeg1 = 29;
  hfdcan2.Init.DataTimeSeg2 = 10;
  hfdcan2.Init.MessageRAMOffset = 0;
  hfdcan2.Init.StdFiltersNbr = 0x00000000;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.RxFifo0ElmtsNbr = 2;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 2;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

static uint32_t HAL_RCC_FDCAN_CLK_ENABLED=0;

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspInit 0 */

  /* USER CODE END FDCAN2_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN2 clock enable */
    HAL_RCC_FDCAN_CLK_ENABLED++;
    if(HAL_RCC_FDCAN_CLK_ENABLED==1){
      __HAL_RCC_FDCAN_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* FDCAN2 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
  /* USER CODE BEGIN FDCAN2_MspInit 1 */

  /* USER CODE END FDCAN2_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
  else if(fdcanHandle->Instance==FDCAN2)
  {
  /* USER CODE BEGIN FDCAN2_MspDeInit 0 */

  /* USER CODE END FDCAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_FDCAN_CLK_ENABLED--;
    if(HAL_RCC_FDCAN_CLK_ENABLED==0){
      __HAL_RCC_FDCAN_CLK_DISABLE();
    }

    /**FDCAN2 GPIO Configuration
    PB5     ------> FDCAN2_RX
    PB6     ------> FDCAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5|GPIO_PIN_6);

    /* FDCAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
  /* USER CODE BEGIN FDCAN2_MspDeInit 1 */

  /* USER CODE END FDCAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void MX_FDCAN1_FilterInit(void)
{
	  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	  
	  FDCAN1_RxFilter.IdType = FDCAN_EXTENDED_ID;   //  ����Ϊ������չ֡
	  FDCAN1_RxFilter.FilterIndex = 0;              //  ��������������
	  FDCAN1_RxFilter.FilterType = FDCAN_FILTER_DUAL;  //  ���˷�ʽΪ��Χ������FilterID1~FilterID2֮���ֵ
	  FDCAN1_RxFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	  FDCAN1_RxFilter.FilterID1 = 0x00000000;
	  FDCAN1_RxFilter.FilterID2 = 0x00000000;           // ��չ֡Ϊ29λID����0x1fffffff����������Ϊ��������֡
	  if(HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_RxFilter) != HAL_OK)
		Error_Handler();
	  
	  __HAL_FDCAN_ENABLE_IT(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);

	  HAL_FDCAN_Start(&hfdcan1);
}

void MX_FDCAN2_FilterInit(void)
{
	  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	  
	  FDCAN2_RxFilter.IdType = FDCAN_EXTENDED_ID;   //  ����Ϊ������չ֡
	  FDCAN2_RxFilter.FilterIndex = 0;              //  ��������������
	  FDCAN2_RxFilter.FilterType = FDCAN_FILTER_DUAL;  //  ���˷�ʽΪ��Χ������FilterID1~FilterID2֮���ֵ
	  FDCAN2_RxFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	  FDCAN2_RxFilter.FilterID1 = 0x00000000;
	  FDCAN2_RxFilter.FilterID2 = 0x00000000;           // ��չ֡Ϊ29λID����0x1fffffff����������Ϊ��������֡
	  if(HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_RxFilter) != HAL_OK)
		Error_Handler();
	  
	  __HAL_FDCAN_ENABLE_IT(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE);

	  HAL_FDCAN_Start(&hfdcan2);
}

uint8_t FDCAN1_Send_Msg(uint8_t* msg,uint32_t id)
{	
	fdcan1_TxHeader.Identifier = id;
	fdcan1_TxHeader.IdType=FDCAN_STANDARD_ID;                  //��׼ID
	fdcan1_TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //����֡
	fdcan1_TxHeader.DataLength=FDCAN_DLC_BYTES_8;              //���ݳ���
	fdcan1_TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
	fdcan1_TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //�ر������л�
	fdcan1_TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
	fdcan1_TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //�޷����¼�
	fdcan1_TxHeader.MessageMarker=0x52;                           
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&fdcan1_TxHeader,msg);
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1,&fdcan1_TxHeader,msg)!=HAL_OK) return 1;//����
	return 0;	
}

uint8_t FDCAN2_Send_Msg(uint8_t* msg,uint32_t id)
{	
	fdcan2_TxHeader.Identifier = id;
	fdcan2_TxHeader.IdType=FDCAN_STANDARD_ID;                  //��׼ID
	fdcan2_TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //����֡
	fdcan2_TxHeader.DataLength=FDCAN_DLC_BYTES_8;              //���ݳ���
	fdcan2_TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
	fdcan2_TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //�ر������л�
	fdcan2_TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //��ͳ��CANģʽ
	fdcan2_TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //�޷����¼�
	fdcan2_TxHeader.MessageMarker=0x52;                           
	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&fdcan2_TxHeader,msg);
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2,&fdcan2_TxHeader,msg)!=HAL_OK) return 1;//����
	return 0;	
}

uint8_t FDCAN1_Receive_Msg(uint8_t *buf)
{
	if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0) return 0; /* ���û�н��յ����ݳ���Ϊ0 */
	
	HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &fdcan1_RxHeader, buf);
	
	return fdcan1_RxHeader.Identifier;
}

uint8_t FDCAN2_Receive_Msg(uint8_t *buf)
{
	if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) == 0) return 0; /* ���û�н��յ����ݳ���Ϊ0 */
	
	HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &fdcan2_RxHeader, buf);
	
	return fdcan2_RxHeader.Identifier;
}

/* USER CODE END 1 */
