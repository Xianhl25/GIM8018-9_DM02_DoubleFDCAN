#ifndef JY901B_H
#define JY901B_H

#include "stdint.h"

#define RXBUFFERSIZE  255     //最大接收字节数
#define TXBUFFERSIZE  255     //最大接收字节数
char RxBuffer[RXBUFFERSIZE];   //接收数据
char TxBuffer[TXBUFFERSIZE];   //接收数据
uint8_t aRxBuffer;			//接收中断缓冲
uint8_t Uart1_Rx_Cnt = 0;		//接收缓冲计数

uint8_t RxSucceeflag = 0;				//	接收成功标记位

float aX = 0;			//	X轴上的加速度
float aY = 0;			//	Y轴上的加速度
float aZ = 0;			//	Z轴上的加速度

float wX = 0;			//	X轴上的角速度
float wY = 0;			//	Y轴上的角速度
float wZ = 0;			//	Z轴上的角速度

float RollX = 0;	//	滚转角
float PitchY = 0;	//	俯仰角
float YawZ	= 0;	//	偏航角

float Height = 0;   //  高度

int checkSum(char RxBuffer[]);


#endif
