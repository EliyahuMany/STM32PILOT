#include "remotecontroller.h"
#include "uart.h"
#include "utils.h"



/**
using timers input capture mode
need to notice if the device works on 5V - if so, need to use FT pins
registers map:
TIM2:
NOT FT - PA0-TIM2_CH1, PA1-TIM2_CH2, PA2-TIM2_CH3, PA3-TIM2_CH4
FT - PA15-TIM2_CH1(remap), PB3-TIM2_CH2(remap), PB10-TIM2_CH3(remap), PB11-TIM2_CH4(remap)
TIM3:
NOT FT - PA6-TIM3_CH1, PA7-TIM3_CH2 or PB5-TIM3_CH2(remap), PB0-TIM3_CH3, PB1-TIM3_CH4
FT - PB4-TIM3_CH1(remap)
TIM4:
FT - PB6-TIM4_CH1, PB7-TIM4_CH2, PB8-TIM4_CH3, PB9-TIM4_CH4
*/

volatile int32_t chan_start, chan_end, pulse_width = 0;
bool firsttime = true;
uint8_t channel_counter = 0;
bool foundSync = false;

void RemoteController::enableTimerChannel(uint8_t channel)
{
	channel -= 1;
	TIM_TypeDef *timer = this->channels[channel].timer;
	uint8_t timerChannel = this->channels[channel].timerChannel + 1; // +1 because when configuring the channels the user sends 1 to N, and in the configuration it is stored in array of 0 to N-1
	if (timer == TIM4 || timer == TIM2 || timer == TIM3)
	{
		if (timer == TIM2) // needed while changing TIMER registers
			NVIC_DisableIRQ(TIM2_IRQn);
		else if (timer == TIM3) // needed while changing TIMER registers
			NVIC_DisableIRQ(TIM3_IRQn);
		else if (timer == TIM4) // needed while changing TIMER registers
			NVIC_DisableIRQ(TIM4_IRQn);
		switch(timerChannel)
		{
			case 1:
				timer->DIER |= TIM_DIER_CC1IE;
				timer->CCER &= ~TIM_CCER_CC1E;
				timer->CCMR1 &= ~TIM_CCMR1_CC1S_1;
				timer->CCMR1 |= TIM_CCMR1_CC1S_0;
				timer->CCER |= TIM_CCER_CC1E;
				break;
			case 2:
				timer->DIER |= TIM_DIER_CC2IE;
				timer->CCER &= ~TIM_CCER_CC2E;
				timer->CCMR1 &= ~TIM_CCMR1_CC2S_1;
				timer->CCMR1 |= TIM_CCMR1_CC2S_0;
				timer->CCER |= TIM_CCER_CC2E;
				break;
			case 3:
				timer->DIER |= TIM_DIER_CC3IE;
				timer->CCER &= ~TIM_CCER_CC3E;
				timer->CCMR2 &= ~TIM_CCMR2_CC3S_1;
				timer->CCMR2 |= TIM_CCMR2_CC3S_0;
				timer->CCER |= TIM_CCER_CC3E;
				break;
			case 4:
				timer->DIER |= TIM_DIER_CC4IE;
				timer->CCER &= ~TIM_CCER_CC4E;
				timer->CCMR2 &= ~TIM_CCMR2_CC4S_1;
				timer->CCMR2 |= TIM_CCMR2_CC4S_0;
				timer->CCER |= TIM_CCER_CC4E;
				break;
		}
		if (timer == TIM2)
		{
			NVIC_SetPriority(TIM2_IRQn, 0x03);
			NVIC_EnableIRQ(TIM2_IRQn);
		}
		else if (timer == TIM3)
		{
			NVIC_SetPriority(TIM3_IRQn, 0x03);
			NVIC_EnableIRQ(TIM3_IRQn);
		}
		else if (timer == TIM4)
		{
			NVIC_SetPriority(TIM4_IRQn, 0x03);
			NVIC_EnableIRQ(TIM4_IRQn);
		}
		// SET TIMER TO 1US for each pulse
		timer->PSC = 72-1;
		timer->ARR = 0xffff;
		timer->DCR = 0;
	}
}

void RemoteController::RCInit()
{
//	this->configChannel(TIM4, 1, true, this->rcChannel.Aileron);
//	this->configChannel(TIM4, 3, true, this->rcChannel.SwC);
//	this->configChannel(TIM4, 2, true, this->rcChannel.Elevator);
//	this->configChannel(TIM4, 1, true, this->rcChannel.Elevator);
//	this->enableTimerChannel(this->rcChannel.Aileron);
//	this->enableTimerChannel(this->rcChannel.SwC);
//	this->enableTimerChannel(this->rcChannel.Elevator);
//	rc->configChannel(TIM3, 1, true, 5);
//	rc->enableTimerChannel(TIM3, 1);
	
	// using PPM - capable of only 8 channel.
	this->configChannel(TIM4, 1, true, this->rcChannel.Elevator);
	this->configChannel(TIM4, 1, true, this->rcChannel.Aileron);
	this->configChannel(TIM4, 1, true, this->rcChannel.Throttle);
	this->configChannel(TIM4, 1, true, this->rcChannel.Rudder);
	this->configChannel(TIM4, 1, true, this->rcChannel.VarA);
	this->configChannel(TIM4, 1, true, this->rcChannel.VarB);
	this->configChannel(TIM4, 1, true, this->rcChannel.SwA);
	this->configChannel(TIM4, 1, true, this->rcChannel.SwB);
	this->enableTimerChannel(this->rcChannel.Elevator); // sending only one of the above - because all using the same Timer Channel.
	
	
}

void RemoteController::pp()
{
	for (int i=0; i<10; i++)
	{
		printf("chan%d: %d", i+1, this->channels[i].pulseWidth);
		if (i != 9)
			printf(",");
	}
	printf("\n");
}

int32_t RemoteController::getChannelPWM(uint8_t channel)
{
	channel -= 1;
	return constrain(this->channels[channel].pulseWidth, 1000, 2000);
}

void RemoteController::chan_counter(uint8_t i) // i IS THE REMOTE CONTROL CHANNEL (not the channel of the timers)
{
	i = i-1; // the function called with number of channel (1 to N), but i treated as array position
	uint32_t timerNBaseAddress = (uint32_t)this->channels[i].timer;
	uint8_t timerChannelOffset = this->channels[i].timerChannel*4;
	uint32_t ccrValue = (*((uint32_t *)(timerNBaseAddress + CCR_BEGIN_ADDRESS + timerChannelOffset)));
	if (this->channels[i].gpioPort->IDR & this->channels[i].gpioPin)
	{
		this->channels[i].start = ccrValue;
		this->channels[i].timer->CCER |= (TIM_CCER_CC1P << (this->channels[i].timerChannel*4)); // switch to falling edge (shifting because on all timers it has the same register structure, so in order to use only 1 function it was neccesery to add it)
	}
	else
	{
		this->channels[i].pulseWidth = ccrValue - this->channels[i].start;
		if (this->channels[i].pulseWidth < 0)
			this->channels[i].pulseWidth += 0xffff;
		this->channels[i].timer->CCER &= ~(TIM_CCER_CC1P << (this->channels[i].timerChannel*4)); // switch to rising edge
	}

}

void RemoteController::ppm_counter(uint8_t i) // i IS THE REMOTE CONTROL CHANNEL (not the channel of the timers)
{
	i = i-1; // the function called with number of channel (1 to N), but i treated as array position
	uint32_t timerNBaseAddress = (uint32_t)this->channels[i].timer;
	uint8_t timerChannelOffset = this->channels[i].timerChannel*4;
	
	
	if (this->channels[i].timer->SR & (1 << (this->channels[i].timerChannel +2)))
	{
		uint32_t ccrValue = (*((uint32_t *)(timerNBaseAddress + CCR_BEGIN_ADDRESS + timerChannelOffset)));
		if (firsttime)
		{
			chan_start = ccrValue;
			firsttime = false;
			channel_counter = 0;
		}
		else
		{
			channel_counter += 1;
			chan_end = ccrValue;
			pulse_width = chan_end - chan_start;
			chan_start = chan_end;
			
			if (pulse_width < 0)
				pulse_width += 0xffff;
			if (pulse_width > 2100)
			{
				// found sync
				if (!foundSync)
					foundSync = true;
				channel_counter = 0;
			}
			else if (foundSync)
			{
				// store pulse width
				this->channels[channel_counter-1].pulseWidth = constrain(pulse_width, 1000, 2000);
			}
		}
	}

}
