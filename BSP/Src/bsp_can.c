#include "fdcan.h"
#include "bsp_can.h"
//#include "Motor.h"

FDCAN_RxFrame_TypeDef FDCAN1_RxFrame;
FDCAN_RxFrame_TypeDef FDCAN2_RxFrame;

uint8_t bit8tofloat[4];

FDCAN_TxFrame_TypeDef SteerTxFrame = {
  .hcan = &hfdcan2,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};
FDCAN_TxFrame_TypeDef WheelTxFrame = {
  .hcan = &hfdcan1,
  .Header.IdType = FDCAN_STANDARD_ID, 
  .Header.TxFrameType = FDCAN_DATA_FRAME,
  .Header.DataLength = 8,
	.Header.ErrorStateIndicator =  FDCAN_ESI_ACTIVE,
  .Header.BitRateSwitch = FDCAN_BRS_OFF,
  .Header.FDFormat =  FDCAN_CLASSIC_CAN,           
  .Header.TxEventFifoControl =  FDCAN_NO_TX_EVENTS,  
  .Header.MessageMarker = 0,
};

void BSP_FDCAN_Init(void){

  FDCAN_FilterTypeDef FDCAN1_FilterConfig;
	
	FDCAN1_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN1_FilterConfig.FilterIndex = 0;
  FDCAN1_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN1_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  FDCAN1_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN1_FilterConfig.FilterID2 = 0x00000000; 
  
	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  
	
	 FDCAN_FilterTypeDef FDCAN2_FilterConfig;
	
	FDCAN2_FilterConfig.IdType = FDCAN_STANDARD_ID;
  FDCAN2_FilterConfig.FilterIndex = 0;
  FDCAN2_FilterConfig.FilterType = FDCAN_FILTER_MASK;
  FDCAN2_FilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  FDCAN2_FilterConfig.FilterID1 = 0x00000000; 
  FDCAN2_FilterConfig.FilterID2 = 0x00000000; 
  
	if(HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
		
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }
 
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}



static void FDCAN1_RxFifo0RxHandler(uint32_t *StdId,uint8_t Data[8])
{
  
//	switch(Data[0]&0x0F){
//		
//	 case 0x01 :	
//	 
//	 DM_Motor_Info_Update(Data,&DM_6215_Motor[0]);
//	
//	 break;
//  
//	 case 0x02 :	
//	 
//	 DM_Motor_Info_Update(Data,&DM_6215_Motor[1]);
//	
//	 break;
//		
//	 case 0x03 :	
//	 
//	 DM_Motor_Info_Update(Data,&DM_6215_Motor[2]);
//	
//	 break;

//   case 0x04 :	
//	 
//	 DM_Motor_Info_Update(Data,&DM_6215_Motor[3]);
//	
//	 break;	 
//	 
//   default:
//	  break;
// }


}

static void FDCAN2_RxFifo1RxHandler(uint32_t *StdId,uint8_t Data[8])
{

//	switch(Data[0]&0x0F){
//		
//	case 0x01 :	
//		
//	 DM_Motor_Info_Update(Data,&DM_6220_Motor[0]);
//	
//	break;
//	
//	case 0x02 :
//		
//	 DM_Motor_Info_Update(Data,&DM_6220_Motor[1]);
//	
//	break;
//	
//	case 0x03 :
//		
//	 DM_Motor_Info_Update(Data,&DM_6220_Motor[2]);
//	
//	break;
//	
//	case 0x04 :
//		
//	 DM_Motor_Info_Update(Data,&DM_6220_Motor[3]);
//	
//	break;
//	
//	default:
//	break;
//	}
	
	

}
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN1_RxFrame.Header, FDCAN1_RxFrame.Data);
	
  FDCAN1_RxFifo0RxHandler(&FDCAN1_RxFrame.Header.Identifier,FDCAN1_RxFrame.Data);
	 
}
	
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{ 
  
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &FDCAN2_RxFrame.Header, FDCAN2_RxFrame.Data);
	
  FDCAN2_RxFifo1RxHandler(&FDCAN2_RxFrame.Header.Identifier,FDCAN2_RxFrame.Data);
	 
}
	
	
	
