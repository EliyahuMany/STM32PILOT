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
	registerStruct->CR1 |= USART_CR1_UE; /*enable usart*/
}

int	uartHandler::read(void) {
	while(!(this->registerStruct->SR & USART_SR_RXNE)){} // wait until Rx empty
	return this->registerStruct->DR;
}

int uartHandler::write(int ch) {
	while(!(this->registerStruct->SR & USART_SR_TXE)){} // wait until Tx empty
	this->registerStruct->DR = (ch & 0xFF);
	return 1;
}

int uartHandler::Scanf(char *ptr, int len)
{
	bool foundStart = false;
  int DataIdx = 0;
	int c;
  while(DataIdx < len)
  {
		//GPIOB->ODR ^= GPIO_ODR_ODR14;
		c = this->read();
		if (foundStart)
		{
			*ptr++=c;
			DataIdx+=1;
		}
		else
			if (c == '$')
			{
				foundStart = true;
			}
		//GPIOB->ODR ^= GPIO_ODR_ODR14;
  }
  return DataIdx;
}

void uartHandler::sendStream(size_t len, char *arr)
{
 while(len--)
 {
	 this->write((int)*arr++);
 }
}


// temporarly use printf to send streams of mixed types, TODO: fix sendStream
int uart_write(int ch) {
	while(!(USART1->SR & USART_SR_TXE)){} // wait until Tx empty
	USART1->DR = (ch & 0xFF);
	return 1;
		
}


namespace std{
	struct __FILE
	{ int handle;};
	FILE __stdout;
	FILE __stdin;
	FILE __stderr;
	
	
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
