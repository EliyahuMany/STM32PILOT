#include "SysTickHandler.h"

volatile uint32_t msTicks = 0;

extern "C" void SysTick_Handler(void)
{
	msTicks += 1;
}


uint32_t GetTick()
{
	return msTicks;
}
