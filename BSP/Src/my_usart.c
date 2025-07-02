#include "my_usart.h"

#define RXBUFFERSIZE  255     //�������ֽ���
#define TXBUFFERSIZE  255     //�������ֽ���
extern char RxBuffer[RXBUFFERSIZE];   //��������
extern char TxBuffer[TXBUFFERSIZE];   //��������
extern uint8_t aRxBuffer;			//�����жϻ���
extern uint8_t Uart1_Rx_Cnt;		//���ջ������
extern uint8_t RxSucceeflag;				//	���ճɹ����λ
extern float aX;			//	X���ϵļ��ٶ�
extern float aY;			//	Y���ϵļ��ٶ�
extern float aZ;			//	Z���ϵļ��ٶ�
extern float wX;			//	X���ϵĽ��ٶ�
extern float wY;			//	Y���ϵĽ��ٶ�
extern float wZ;			//	Z���ϵĽ��ٶ�
extern float RollX;	//	��ת��
extern float PitchY;	//	������
extern float YawZ;	//	ƫ����
extern float Height;


int fputc(int _char, FILE *_stream)
{
    /* �����������ͺ��� ����1 ����������  �������ݸ���  ���ͳ�ʱʱ��*/
    HAL_UART_Transmit(&huart10, (const uint8_t *)&_char, 1,1000);
    
    return _char;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
		char tempBuffer[100] = "";   // �м�ת������
		char i = 0; 								 //	ѭ������	
		/* Prevent unused argument(s) compilation warning */
		UNUSED(huart);
		/* NOTE: This function Should not be modified, when the callback is needed,
						 the HAL_UART_TxCpltCallback could be implemented in the user file
		 */
		
		if(huart == &huart7)
		{	
			//while(HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *)RxBuffer, 44,0xFFFF));			//	���ָ������
			
			if(0X55 != RxBuffer[0] || 0X51 != RxBuffer[1])
			{
				memset(RxBuffer,0x00,sizeof(RxBuffer)); //�������
//				while(HAL_OK != HAL_UART_Transmit(&huart10, (uint8_t *)"ERROR", 5,0xFFFF));			//	���ָ������
				while(HAL_OK != HAL_UART_Receive_IT(&huart7, (uint8_t *)RxBuffer, 55));   //���������жϣ�����֤�����ɹ� 
				return;
			}
			RxSucceeflag = 1;					//	���ݳɹ����ձ�־
			if(0X51 == RxBuffer[1])
			{
				memset(tempBuffer,0x00,sizeof(tempBuffer)); //�������
				for(i=0;i<11;i++)
				{
					tempBuffer[i] = RxBuffer[i]; 
				}
				if(1 == checkSum(tempBuffer))
				{
					aX = (float)((float)((RxBuffer[3]<<8)|RxBuffer[2])/32768.0*16);
					aY = (float)((float)((RxBuffer[5]<<8)|RxBuffer[4])/32768.0*16);
					aZ = (float)((float)((RxBuffer[7]<<8)|RxBuffer[6])/32768.0*16);
					
					//while(HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *)"���ٶ�\r\n", strlen("���ٶ�\r\n"),0xFFFF));
				}
			}
			if(0X52 == RxBuffer[12])
			{
				memset(tempBuffer,0x00,sizeof(tempBuffer)); //�������
				for(i=11;i<22;i++)
				{
					tempBuffer[i-11] = RxBuffer[i]; 
				}
				
				if(1 == checkSum(RxBuffer))
				{
					wX = (float)(((RxBuffer[14]<<8)|RxBuffer[13])/32768.0*2000);
					wY = (float)(((RxBuffer[16]<<8)|RxBuffer[15])/32768.0*2000);
					wZ = (float)(((RxBuffer[18]<<8)|RxBuffer[17])/32768.0*2000);
					
					//while(HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *)"���ٶ�\r\n", strlen("���ٶ�\r\n"),0xFFFF));
				}
			}
			if(0X53 == RxBuffer[23])
			{
				memset(tempBuffer,0x00,sizeof(tempBuffer)); //�������
				for(i=22;i<33;i++)
				{
					tempBuffer[i-22] = RxBuffer[i]; 
				}
				
				if(1 == checkSum(RxBuffer))
				{
					RollX = (float)(((RxBuffer[25]<<8)|RxBuffer[24])/32768.0*180);
					PitchY = (float)(((RxBuffer[27]<<8)|RxBuffer[26])/32768.0*180);
					YawZ = (float)(((RxBuffer[29]<<8)|RxBuffer[28])/32768.0*180);
					
					//while(HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *)"��ת��\r\n", strlen("��ת��\r\n"),0xFFFF));
				}
			}
			if(0X56 == RxBuffer[45])
			{
				memset(tempBuffer,0x00,sizeof(tempBuffer)); //�������
				for(i=44;i<55;i++)
				{
					tempBuffer[i-44] = RxBuffer[i]; 
				}
				
				if(1 == checkSum(RxBuffer))
				{
					Height = (RxBuffer[53]<<24)|(RxBuffer[52]<<16)|(RxBuffer[51]<<8)|RxBuffer[50];
					
					//while(HAL_OK != HAL_UART_Transmit(&huart1, (uint8_t *)"��ת��\r\n", strlen("��ת��\r\n"),0xFFFF));
				}
			}
			memset(RxBuffer,0x00,sizeof(RxBuffer)); //�������
			while(HAL_OK != HAL_UART_Receive_IT(&huart7, (uint8_t *)RxBuffer, 55));   //���������жϣ�����֤�����ɹ� 
		}
			
}

//���ڴ�ӡ
void send_string(const char *str) {
    HAL_UART_Transmit(&huart10, (uint8_t *)str, strlen(str), 0xFFFF); // �����ַ���
}

//���ڴ�ӡ
void send_int(int value) {
    char buffer[20];
    sprintf(buffer, "%d", value); // ������ת�����ַ���
    send_string(buffer);
}

//���ڴ�ӡ
void send_float(float value) {
    char buffer[20];
    sprintf(buffer, "%.2f", value); // ��������ת�����ַ�����������λС��
    send_string(buffer);
}

//���ڴ�ӡ
void print_data(const char *prefix, float float_data) {
    send_string(prefix); // ����̶��ַ���
    send_float(float_data); // �����������
    send_string("\r\n"); // ����
}

