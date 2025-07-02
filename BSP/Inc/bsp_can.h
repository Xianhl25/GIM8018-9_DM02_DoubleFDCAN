#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "stm32h7xx.h"


extern void BSP_FDCAN_Init(void);

//typedef struct {
//	FDCAN_HandleTypeDef *hcan;
//    FDCAN_TxHeaderTypeDef Header;
//    uint8_t				Data[8];
//}FDCAN_TxFrame_TypeDef;

//typedef struct {
//	FDCAN_HandleTypeDef *hcan;
//    FDCAN_RxHeaderTypeDef Header;
//    uint8_t 			Data[8];
//} FDCAN_RxFrame_TypeDef;


//extern  FDCAN_TxFrame_TypeDef   SteerTxFrame;
//extern  FDCAN_TxFrame_TypeDef   WheelTxFrame;



extern  uint8_t bit8tofloat[4];

uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
	   
#endif