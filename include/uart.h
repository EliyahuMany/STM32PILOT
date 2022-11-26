
#ifndef __UART_H
#define __UART_H
#include "stm32f10x.h"                  // Device header
#include <stdio.h>

void	uart_init(void);
int		uart_write(int ch);
int		uart_read(void);

class uartHandler
{
	public:
		USART_TypeDef* registerStruct;
		uartHandler(USART_TypeDef* registerStruct, uint32_t baudRate);
		int read();
		int write(int ch);
};
#endif