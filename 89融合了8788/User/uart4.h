#ifndef __USART4_H
#define	__USART4_H


#include "stm32f10x.h"
#include <stdio.h>

//���ڽ��ջ��������С
#define UART_RX_BUFFER_SIZE4 256 
void UART_Config4(void);
extern unsigned char UART_RxBuffer4[UART_RX_BUFFER_SIZE4];
#endif


