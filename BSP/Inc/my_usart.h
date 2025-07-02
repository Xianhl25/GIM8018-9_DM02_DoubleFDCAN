#ifndef MY_USART_H
#define MY_USART_H
#include "stdio.h"
#include "usart.h"
#include "string.h"

int fputc(int _char, FILE *_stream);

void send_string(const char *str) ;
void send_int(int value) ;
void send_float(float value);
void print_data(const char *prefix, float float_data);
	

extern void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);


#endif