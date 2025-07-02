#ifndef __CAN_COMM_H
#define __CAN_COMM_H

#include "stm32h7xx_hal.h"

#define CMD_MOTOR_MODE      0x01
#define CMD_RESET_MODE      0x02
#define CMD_ZERO_POSITION   0x03

#define CAN_SLAVE_ID        0x01

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#define P_MIN -95.5f    // Radians
#define P_MAX 95.5f        
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f
#define KD_MIN 0.0f     // N-m/rad/s
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void Can1Comm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t);
void Can2Comm_SendControlPara(float f_p, float f_v, float f_kp, float f_kd, float f_t);
void Can1Comm_ControlCmd(uint8_t cmd);
void Can2Comm_ControlCmd(uint8_t cmd);

#endif
