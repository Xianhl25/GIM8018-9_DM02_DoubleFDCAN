#include "JY901B.h"

int checkSum(char RxBuffer[])
{
	int i;				//	循环变量
	int sum = 0;
	for(i=0;i<10;i++)
	{
		sum = sum+RxBuffer[i];
	}
	if(RxBuffer[10] == (char)(sum))
	{
		return 1;				//	返回1表示数据正确
	}
	else
	{
		return -1;				//	返回-1表示数据不正确
	}
}