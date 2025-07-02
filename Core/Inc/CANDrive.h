#ifndef _CANDrive_H_
#define _CANDrive_H_

#include "stm32h7xx_hal.h"

#ifdef HAL_FDCAN_MODULE_ENABLED

#include "fdcan.h"

extern uint8_t FDCAN1_buff[8];        //!<@brief FDCAN1���ջ�����


/**
 * @brief ����ͨ�����ó�ʼ��CAN�˲���
 * @param hfdcan CAN handle Structure definition
 */
void CanFilter_Init(void);

/**
 * @brief CAN���ͱ�׼֡����
 * @param hfdcan CAN���
 * @param[in] Identifier ID
 * @param[in] msg ��������,����Ϊ8
 * @return HAL Status structures definition
 */
HAL_StatusTypeDef CAN_Send_StdDataFrame(FDCAN_HandleTypeDef *hfdcan, uint32_t Identifier, uint8_t *msg);

/**
 * @brief CAN��ȡ����
 * @param hcan CAN���
 * @param[out] buf ���ݻ�����
 * @return ��׼֡ID����չ֡ID
 */
uint32_t CAN_Receive_DataFrame(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf);


#endif //HAL_FDCAN_MODULE_ENABLED

#endif //_CANDrive_H_
