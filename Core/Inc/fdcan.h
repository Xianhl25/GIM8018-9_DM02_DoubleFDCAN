/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.h
  * @brief   This file contains all the function prototypes for
  *          the fdcan.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern FDCAN_HandleTypeDef hfdcan1;

extern FDCAN_HandleTypeDef hfdcan2;

/* USER CODE BEGIN Private defines */
typedef struct {
	FDCAN_HandleTypeDef *hcan;
	FDCAN_TxHeaderTypeDef Header;
	uint8_t Data[8];
}FDCAN_TxFrame_TypeDef;

typedef struct {
	FDCAN_HandleTypeDef *hcan;
	FDCAN_RxHeaderTypeDef Header;
	uint8_t Data[8];
}FDCAN_RxFrame_TypeDef;

typedef FDCAN_HandleTypeDef hcan_t;

/* USER CODE END Private defines */

void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);

/* USER CODE BEGIN Prototypes */
void MX_FDCAN1_FilterInit(void);
void MX_FDCAN2_FilterInit(void);

uint8_t FDCAN1_Send_Msg(uint8_t* msg,uint32_t id);
uint8_t FDCAN2_Send_Msg(uint8_t* msg,uint32_t id);

uint8_t FDCAN1_Receive_Msg(uint8_t *buf);
uint8_t FDCAN2_Receive_Msg(uint8_t *buf);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */

