#include "JY901B.h"

int checkSum(char RxBuffer[])
{
	int i;				//	ѭ������
	int sum = 0;
	for(i=0;i<10;i++)
	{
		sum = sum+RxBuffer[i];
	}
	if(RxBuffer[10] == (char)(sum))
	{
		return 1;				//	����1��ʾ������ȷ
	}
	else
	{
		return -1;				//	����-1��ʾ���ݲ���ȷ
	}
}