#ifndef _CANDrive_H_
#define _CANDrive_H_

#include "stm32h7xx_hal.h"

#ifdef HAL_FDCAN_MODULE_ENABLED

#include "fdcan.h"

extern uint8_t FDCAN1_buff[8];        //!<@brief FDCAN1接收缓冲区


/**
 * @brief 按照通常设置初始化CAN滤波器
 * @param hfdcan CAN handle Structure definition
 */
void CanFilter_Init(void);

/**
 * @brief CAN发送标准帧数据
 * @param hfdcan CAN句柄
 * @param[in] Identifier ID
 * @param[in] msg 数据数组,长度为8
 * @return HAL Status structures definition
 */
HAL_StatusTypeDef CAN_Send_StdDataFrame(FDCAN_HandleTypeDef *hfdcan, uint32_t Identifier, uint8_t *msg);

/**
 * @brief CAN读取数据
 * @param hcan CAN句柄
 * @param[out] buf 数据缓冲区
 * @return 标准帧ID或拓展帧ID
 */
uint32_t CAN_Receive_DataFrame(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf);


#endif //HAL_FDCAN_MODULE_ENABLED

#endif //_CANDrive_H_
