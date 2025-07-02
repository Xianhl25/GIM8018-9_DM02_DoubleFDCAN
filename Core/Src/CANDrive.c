#include "CANDrive.h"

#ifdef HAL_FDCAN_MODULE_ENABLED

uint8_t FDCAN1_buff[8];

void CanFilter_Init(void) {
    FDCAN_FilterTypeDef sFilterConfig;
    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x000007FF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    sFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x1FFFFFFF;
    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }
}

HAL_StatusTypeDef CAN_Send_StdDataFrame(FDCAN_HandleTypeDef *hfdcan, uint32_t Identifier, uint8_t *msg) {
    FDCAN_TxHeaderTypeDef CAN_Tx = {
            .Identifier = Identifier,                 //标准标识符
            .IdType = FDCAN_STANDARD_ID,              //使用标准帧
            .TxFrameType = FDCAN_DATA_FRAME,          //数据帧
            .DataLength = FDCAN_DLC_BYTES_8,
            .ErrorStateIndicator = FDCAN_ESI_PASSIVE,
            .FDFormat = FDCAN_CLASSIC_CAN,
            .BitRateSwitch = FDCAN_BRS_OFF,
            .TxEventFifoControl=FDCAN_NO_TX_EVENTS,
            .MessageMarker=0
            };
    HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, hfdcan1.Init.DataPrescaler * hfdcan1.Init.DataTimeSeg1, 0);
    HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);

    HAL_StatusTypeDef err = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &CAN_Tx, msg);
    return err;
}

uint32_t CAN_Receive_DataFrame(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf) {
    FDCAN_RxHeaderTypeDef CAN_Rx = { 0 };
    if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN_Rx, buf) == HAL_OK) {
        return CAN_Rx.Identifier;        
    }
    else {
        Error_Handler();
        return 0;
    }
}

#endif
