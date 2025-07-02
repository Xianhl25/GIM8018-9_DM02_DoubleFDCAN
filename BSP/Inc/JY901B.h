#ifndef JY901B_H
#define JY901B_H

#include "stdint.h"

#define RXBUFFERSIZE  255     //�������ֽ���
#define TXBUFFERSIZE  255     //�������ֽ���
char RxBuffer[RXBUFFERSIZE];   //��������
char TxBuffer[TXBUFFERSIZE];   //��������
uint8_t aRxBuffer;			//�����жϻ���
uint8_t Uart1_Rx_Cnt = 0;		//���ջ������

uint8_t RxSucceeflag = 0;				//	���ճɹ����λ

float aX = 0;			//	X���ϵļ��ٶ�
float aY = 0;			//	Y���ϵļ��ٶ�
float aZ = 0;			//	Z���ϵļ��ٶ�

float wX = 0;			//	X���ϵĽ��ٶ�
float wY = 0;			//	Y���ϵĽ��ٶ�
float wZ = 0;			//	Z���ϵĽ��ٶ�

float RollX = 0;	//	��ת��
float PitchY = 0;	//	������
float YawZ	= 0;	//	ƫ����

float Height = 0;   //  �߶�

int checkSum(char RxBuffer[]);


#endif
