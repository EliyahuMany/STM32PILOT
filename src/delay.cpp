#include "delay.h"




void delay(int delayTime)
{
	// check system clock speed (assuming using internal 8MHz):
	uint32_t pllMul = ((RCC->CFGR & RCC_CFGR_PLLMULL) >> 18) + 2;
	uint32_t maxTimeOfDelay = 16777216 * (1/(8000000*pllMul));
	// 1 second = (1/(8000000*pllMul))
	SysTick->LOAD = (8000000*pllMul)/1000 - 1;
	//SysTick->LOAD = (SystemCoreClock/1000) - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = 5;
	
	for (int i=0; i<delayTime; i++)
	{
		while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG) == 0); // wait untill the count flag raise to 1
	}
	SysTick->CTRL = 0;
}
