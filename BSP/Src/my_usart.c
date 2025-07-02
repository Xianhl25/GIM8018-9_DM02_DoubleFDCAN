#include "my_usart.h"

#define RXBUFFERSIZE  255     //最大接收字节数
#define TXBUFFERSIZE  255     //最大接收字节数
extern char RxBuffer[RXBUFFERSIZE];   //接收数据
extern char TxBuffer[TXBUFFERSIZE];   //接收数据
extern uint8_t aRxBuffer;			//接收中断缓冲
extern uint8_t Uart1_Rx_Cnt;		//接收缓冲计数
extern uint8_t RxSucceeflag;				//	接收成功标记位
extern float aX;			//	X轴上的加速度
extern float aY;			//	Y轴上的加速度
extern float aZ;			//	Z轴上的加速度
extern float wX;			//	X轴上的角速度
extern float wY;			//	Y轴上的角速度
extern float wZ;			//	Z轴上的角速度
extern float RollX;	//	滚转角
extern float PitchY;	//	俯仰角
extern float YawZ;	//	偏航角
extern float Height;


int fputc(int _char, FILE *_stream)
{
    /* 串口阻塞发送函数 串口1 待发送数据  发送数据个数  发送超时时间*/
    HAL_UART_Transmit(&huart10, (const uint8_t *)&_char, 1,1000);
    
    return _char;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		char tempBuffer[100] = "";   // 中间转存数组
		char i = 0; 								 //	循环变量	
		/* Prevent unused argument(s) compilation warning */
		UNUSED(huart);
		/* NOTE: This function Should not be modified, when the callback is needed,
						 the HAL_UART_TxCpltCallback could be implemented in the user file
		 */
		
		if(huart == &huart7)
		{	
			//while(HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *)RxBuffer, 44,0xFFFF));			//	输出指定长度
			
			if(0X55 != RxBuffer[0] || 0X51 != RxBuffer[1])
			{
				memset(RxBuffer,0x00,sizeof(RxBuffer)); //清空数组
//				while(HAL_OK != HAL_UART_Transmit(&huart10, (uint8_t *)"ERROR", 5,0xFFFF));			//	输出指定长度
				while(HAL_OK != HAL_UART_Receive_IT(&huart7, (uint8_t *)RxBuffer, 55));   //开启接收中断，并保证开启成功 
				return;
			}
			RxSucceeflag = 1;					//	数据成功接收标志
			if(0X51 == RxBuffer[1])
			{
				memset(tempBuffer,0x00,sizeof(tempBuffer)); //清空数组
				for(i=0;i<11;i++)
				{
					tempBuffer[i] = RxBuffer[i]; 
				}
				if(1 == checkSum(tempBuffer))
				{
					aX = (float)((float)((RxBuffer[3]<<8)|RxBuffer[2])/32768.0*16);
					aY = (float)((float)((RxBuffer[5]<<8)|RxBuffer[4])/32768.0*16);
					aZ = (float)((float)((RxBuffer[7]<<8)|RxBuffer[6])/32768.0*16);
					
					//while(HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *)"加速度\r\n", strlen("加速度\r\n"),0xFFFF));
				}
			}
			if(0X52 == RxBuffer[12])
			{
				memset(tempBuffer,0x00,sizeof(tempBuffer)); //清空数组
				for(i=11;i<22;i++)
				{
					tempBuffer[i-11] = RxBuffer[i]; 
				}
				
				if(1 == checkSum(RxBuffer))
				{
					wX = (float)(((RxBuffer[14]<<8)|RxBuffer[13])/32768.0*2000);
					wY = (float)(((RxBuffer[16]<<8)|RxBuffer[15])/32768.0*2000);
					wZ = (float)(((RxBuffer[18]<<8)|RxBuffer[17])/32768.0*2000);
					
					//while(HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *)"角速度\r\n", strlen("角速度\r\n"),0xFFFF));
				}
			}
			if(0X53 == RxBuffer[23])
			{
				memset(tempBuffer,0x00,sizeof(tempBuffer)); //清空数组
				for(i=22;i<33;i++)
				{
					tempBuffer[i-22] = RxBuffer[i]; 
				}
				
				if(1 == checkSum(RxBuffer))
				{
					RollX = (float)(((RxBuffer[25]<<8)|RxBuffer[24])/32768.0*180);
					PitchY = (float)(((RxBuffer[27]<<8)|RxBuffer[26])/32768.0*180);
					YawZ = (float)(((RxBuffer[29]<<8)|RxBuffer[28])/32768.0*180);
					
					//while(HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *)"滚转角\r\n", strlen("滚转角\r\n"),0xFFFF));
				}
			}
			if(0X56 == RxBuffer[45])
			{
				memset(tempBuffer,0x00,sizeof(tempBuffer)); //清空数组
				for(i=44;i<55;i++)
				{
					tempBuffer[i-44] = RxBuffer[i]; 
				}
				
				if(1 == checkSum(RxBuffer))
				{
					Height = (RxBuffer[53]<<24)|(RxBuffer[52]<<16)|(RxBuffer[51]<<8)|RxBuffer[50];
					
					//while(HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *)"滚转角\r\n", strlen("滚转角\r\n"),0xFFFF));
				}
			}
			memset(RxBuffer,0x00,sizeof(RxBuffer)); //清空数组
			while(HAL_OK != HAL_UART_Receive_IT(&huart7, (uint8_t *)RxBuffer, 55));   //开启接收中断，并保证开启成功 
		}
			
}

//串口打印
void send_string(const char *str) {
    HAL_UART_Transmit(&huart10, (uint8_t *)str, strlen(str), 0xFFFF); // 发送字符串
}

//串口打印
void send_int(int value) {
    char buffer[20];
    sprintf(buffer, "%d", value); // 将整数转换成字符串
    send_string(buffer);
}

//串口打印
void send_float(float value) {
    char buffer[20];
    sprintf(buffer, "%.2f", value); // 将浮点数转换成字符串，保留两位小数
    send_string(buffer);
}

//串口打印
void print_data(const char *prefix, float float_data) {
    send_string(prefix); // 输出固定字符串
    send_float(float_data); // 输出整数数据
    send_string("\r\n"); // 换行
}

