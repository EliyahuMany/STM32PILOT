#include "ppmTest.h"

void chan_counter(uint8_t i) // i IS THE REMOTE CONTROL CHANNEL (not the channel of the timers)
{
//	i = i-1; // the function called with number of channel (1 to N), but i treated as array position
//	uint32_t timerNBaseAddress = (uint32_t)this->channels[i].timer;
//	uint8_t timerChannelOffset = this->channels[i].timerChannel*4;
//	uint32_t ccrValue = (*((uint32_t *)(timerNBaseAddress + CCR_BEGIN_ADDRESS + timerChannelOffset)));
//	if (this->channels[i].gpioPort->IDR & this->channels[i].gpioPin)
//	{
//		this->channels[i].start = ccrValue;
//		this->channels[i].timer->CCER |= (TIM_CCER_CC1P << (this->channels[i].timerChannel*4)); // switch to falling edge (shifting because on all timers it has the same register structure, so in order to use only 1 function it was neccesery to add it)
//	}
//	else
//	{
//		this->channels[i].pulseWidth = ccrValue - this->channels[i].start;
//		if (this->channels[i].pulseWidth < 0)
//			this->channels[i].pulseWidth += 0xffff;
//		this->channels[i].timer->CCER &= ~(TIM_CCER_CC1P << (this->channels[i].timerChannel*4)); // switch to rising edge
//	}
}