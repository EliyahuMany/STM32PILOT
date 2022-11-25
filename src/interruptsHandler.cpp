#include "interruptsHandler.h"


InterruptsHandler::InterruptsHandler()
{
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
}


/**
@param pin - pin number (0-15)
@param port - one of the ports that declared on the mcu system definitions.
the function will set the currect EXTICRx in afio register with the corresponding bit value

****************************************************************************************
************************************* IMPORTANT ****************************************
the user must remmber that he cant set interrupt on DIFFERENT PORTS with SAME PIN number
****************************************************************************************
****************************************************************************************
*/
void InterruptsHandler::configPin(uint8_t pin, GPIO_TypeDef *port)
{
	if (port == GPIOA)
		AFIO->EXTICR[pin/4] |= AFIO_EXTICR_PA << ((pin%4)*4);
	else if (port == GPIOB)
		AFIO->EXTICR[pin/4] |= AFIO_EXTICR_PB << ((pin%4)*4);
	else if (port == GPIOC)
		AFIO->EXTICR[pin/4] |= AFIO_EXTICR_PC << ((pin%4)*4);
	else if (port == GPIOD)
		AFIO->EXTICR[pin/4] |= AFIO_EXTICR_PD << ((pin%4)*4);
	else if (port == GPIOE)
		AFIO->EXTICR[pin/4] |= AFIO_EXTICR_PE << ((pin%4)*4);
	else if (port == GPIOF)
		AFIO->EXTICR[pin/4] |= AFIO_EXTICR_PF << ((pin%4)*4);
	else if (port == GPIOG)
		AFIO->EXTICR[pin/4] |= AFIO_EXTICR_PG << ((pin%4)*4);
		
	EXTI->IMR |= (1 << pin);
	EXTI->RTSR |= (1 << pin);
	EXTI->FTSR |= (1 << pin);
	
	if (pin == 0)
	{
		NVIC_SetPriority(EXTI0_IRQn, 0x03);
		NVIC_EnableIRQ(EXTI0_IRQn);
	}
	else if (pin == 1)
	{
		NVIC_SetPriority(EXTI1_IRQn, 0x03);
		NVIC_EnableIRQ(EXTI1_IRQn);
	}
	else if (pin == 2)
	{
		NVIC_SetPriority(EXTI2_IRQn, 0x03);
		NVIC_EnableIRQ(EXTI2_IRQn);
	}
	else if (pin == 3)
	{
		NVIC_SetPriority(EXTI3_IRQn, 0x03);
		NVIC_EnableIRQ(EXTI3_IRQn);
	}
	else if (pin == 4)
	{
		NVIC_SetPriority(EXTI4_IRQn, 0x03);
		NVIC_EnableIRQ(EXTI4_IRQn);
	}
	else if (pin >=5 && pin <= 9)
	{
		NVIC_SetPriority(EXTI9_5_IRQn, 0x0F);
		NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
	else
	{
		NVIC_SetPriority(EXTI15_10_IRQn, 0x0F);
		NVIC_EnableIRQ(EXTI15_10_IRQn);
	}
}
