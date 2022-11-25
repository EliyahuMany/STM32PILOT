#include "uart.h"
#include "SysTickHandler.h"
// PA2 - TX USART2
// PA3 - RX USART2
// GPIOA -> apb2 bit 2
//usartEN -> apb1 bit 17

// pa10 - rx
// pa9 - tx


uartHandler::uartHandler(USART_TypeDef* registerStruct, uint32_t baudRate)
{
	this->registerStruct = registerStruct;
	// FOR NOW - WITHOUT REMAPPING
	if (registerStruct == USART1) // pa9 = tx , pa10 = rx
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		// TX - Full duplex need alternate function push-pull
		GPIOA->CRH |= GPIO_CRH_MODE9;
		GPIOA->CRH &= ~GPIO_CRH_CNF9;
		GPIOA->CRH |= GPIO_CRH_CNF9_1;
		// RX - full duplex need input floating / input pull up
		GPIOA->CRH &= ~GPIO_CRH_MODE10; // 0011 0000 0000 0000
		GPIOA->CRH |= GPIO_CRH_CNF10_0; // floating point
	}
	else if (registerStruct == USART2) // pa2 = tx , pa3 = rx
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		// TX - Full duplex need alternate function push-pull
		GPIOA->CRL |= GPIO_CRL_MODE2;
		GPIOA->CRL &= ~GPIO_CRL_CNF2;
		GPIOA->CRL |= GPIO_CRL_CNF2_1;
		// RX - full duplex need input floating / input pull up
		GPIOA->CRL &= ~GPIO_CRL_MODE3; // 0011 0000 0000 0000
		GPIOA->CRL |= GPIO_CRL_CNF3_0; // floating point
	}
	/**
	baud rate = Sysclock/(16*usartDiv)
	usartDiv is unsigned fixed point
	*/
	uint32_t divMantissa = SystemCoreClock/(16*baudRate);
	uint8_t divFrac = ((double(SystemCoreClock)/(16*baudRate)) - divMantissa)*16;
	registerStruct->BRR = (divMantissa << 4) | divFrac;
	registerStruct->CR1 = USART_CR1_TE | USART_CR1_RE;
	registerStruct->CR2 = 0x0; /*1 stop bit*/
	registerStruct->CR3 = 0x0; /*no flow control*/
	registerStruct->CR1 |= USART_CR1_UE; /*enable usart2*/
}

int	uartHandler::read(void) {
	while(!(this->registerStruct->SR & USART_SR_RXNE)){} // wait until Rx empty
	return this->registerStruct->DR;
}

int uartHandler::write(int ch) {
	// check
	while(!(this->registerStruct->SR & USART_SR_TXE)){} // wait until Tx empty
	this->registerStruct->DR = (ch & 0xFF);
	return 1;
}

void uart_init(void){
	
	RCC->APB2ENR |=0x04; //00000....100 - Enable GPIOA clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	
	// TX - Full duplex need alternate function push-pull
	GPIOA->CRH |= GPIO_CRH_MODE9;
	GPIOA->CRH |= GPIO_CRH_CNF9_1;
	// RX - full duplex need input floating / input pull up
	GPIOA->CRH &= ~GPIO_CRH_MODE10; // 0011 0000 0000 0000
	GPIOA->CRH |= GPIO_CRH_CNF10_1;
	
	USART1->BRR = 0x271; /*9600 baud @ 72MHz*/
	USART1->CR1 = 0x000C; /*enable TX,RX, set to 8-bit data*/
	USART1->CR2 = 0x0; /*1 stop bit*/
	USART1->CR3 = 0x0; /*no flow control*/
	USART1->CR1 |= 0x2000; /*enable usart2*/
	
}

int uart_write(int ch) {
	// check
	while(!(USART1->SR & USART_SR_TXE)){} // wait until Tx empty
	USART1->DR = (ch & 0xFF);
	return 1;
		
}

int	uart_read(void) {
	uint32_t Tickstart = GetTick();
	while(!(USART1->SR & USART_SR_RXNE))
	{
	} // wait until Rx empty
	return USART1->DR;
}

namespace std{
	struct __FILE
	{ int handle;};
	FILE __stdout;
	FILE __stdin;
	FILE __stderr;

	int fgetc(FILE *f) {
		int c;
		c = uart_read();
		if (c == '\r') {
			uart_write(c);
			c = '\n';
		}
		uart_write(c);
		return c;
	}
	
	int fputc(int c, FILE *stream) {
		return uart_write(c);
	}
	
	int ferror(FILE *stream) {
		return 1;
	}
	
	long int ftell(FILE *stream) {
		return 1;
	}
	
	int fclose(FILE *f) {
		return 1;
	}
	
	int fseek(FILE *f, long nPos, int nMode) {
		return 0;
	}
	
	int fflush(FILE *f) {
		return 1;
	}
}
