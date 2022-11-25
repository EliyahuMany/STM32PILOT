#include "timersHandler.h"

timersHandler::timersHandler(TIM_TypeDef **timers, uint8_t arrLen, uint8_t totalChannels)
{
	this->channels = new Channel[totalChannels];
	this->channelsCreated = 0;
	this->tim2_remap = 0x00;
	this->tim4 = false;
	for (int i=0; i<arrLen; i++)
	{
		if (timers[i] == TIM4)
			RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
		else if (timers[i] == TIM2)
			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		else if (timers[i] == TIM3)
			RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
		timers[i]->CR1 = TIM_CR1_CEN;
	}
}

void timersHandler::configChannel(TIM_TypeDef *timer, uint8_t timerChannel, bool fiveVoltTolerance, uint8_t channel)
{
	channel -= 1; // used as array position later
	this->channels[channel].timer = timer;
	this->channels[channel].timerChannel = timerChannel-1; // -1 because later all the registers address are related to ch1 (position 0x00 for the calculations)
	if (timer == TIM2)
	{
		if (fiveVoltTolerance)
		{
			#define TIMER2_EN
			if (timerChannel == 1)
			{
				this->tim2_remap |= 0x01;
				this->channels[channel].gpioPort = GPIOA;
				this->channels[channel].gpioPin = GPIO_IDR_IDR15;
			}
			else if (timerChannel == 2)
			{
				this->tim2_remap |= 0x01;
				this->channels[channel].gpioPort = GPIOB;
				this->channels[channel].gpioPin = GPIO_IDR_IDR3;
			}
			else if (timerChannel == 3)
			{
				this->tim2_remap |= 0x10;
				this->channels[channel].gpioPort = GPIOB;
				this->channels[channel].gpioPin = GPIO_IDR_IDR10;
			}
			else if (timerChannel == 4)
			{
				this->tim2_remap |= 0x10;
				this->channels[channel].gpioPort = GPIOB;
				this->channels[channel].gpioPin = GPIO_IDR_IDR11;
			}
			RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
			AFIO->MAPR |= this->tim2_remap;
		}
		else
		{
			// pa0,pa1,pa2,pa3 - are not 5Volt tolerant.
			this->channels[channel].gpioPort = GPIOA;
			switch (timerChannel)
			{
				case 1:
					this->channels[channel].gpioPin = GPIO_IDR_IDR0;
					break;
				case 2:
					this->channels[channel].gpioPin = GPIO_IDR_IDR1;
					break;
				case 3:
					this->channels[channel].gpioPin = GPIO_IDR_IDR2;
					break;
				case 4:
					this->channels[channel].gpioPin = GPIO_IDR_IDR3;
					break;
			}
			RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
			AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_NOREMAP;
		}
	}
	else if (timer == TIM3)
	{
		#define TIMER3_EN
		if (fiveVoltTolerance)
		{
			/*
			00: No remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1)
			01: Not used
			10: Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1) *****ATTENTION - CH2 IS NOW PB5******
			11: Full remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9)
			
			if ch1 -> need pb4 (10)
			*/
			if (timerChannel == 1)
			{	
				RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
				AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_PARTIALREMAP; // *****ATTENTION - CH2 IS NOW PB5******
				this->channels[channel].gpioPort = GPIOB;
				this->channels[channel].gpioPin = GPIO_IDR_IDR4;
			}
		}
		else
		{
			switch (timerChannel)
			{
				case 1:
					this->channels[channel].gpioPort = GPIOA;
					this->channels[channel].gpioPin = GPIO_IDR_IDR6;
					break;
				case 2:
					this->channels[channel].gpioPort = GPIOA;
					this->channels[channel].gpioPin = GPIO_IDR_IDR7;
					break;
				case 3:
					this->channels[channel].gpioPort = GPIOB;
					this->channels[channel].gpioPin = GPIO_IDR_IDR0;
					break;
				case 4:
					this->channels[channel].gpioPort = GPIOB;
					this->channels[channel].gpioPin = GPIO_IDR_IDR1;
					break;
			}
			RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
			AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_NOREMAP;
		}
	}
	else if (timer == TIM4)
	{
		#define TIMER4_EN
		if (fiveVoltTolerance)
		{
			/*
			FT - PB6-TIM4_CH1, PB7-TIM4_CH2, PB8-TIM4_CH3, PB9-TIM4_CH4
			0: No remap (TIM4_CH1/PB6, TIM4_CH2/PB7, TIM4_CH3/PB8, TIM4_CH4/PB9)
			1: Full remap (TIM4_CH1/PD12, TIM4_CH2/PD13, TIM4_CH3/PD14, TIM4_CH4/PD15)
			*/
			this->channels[channel].gpioPort = GPIOB;
			switch (timerChannel)
			{
				case 1:
					this->channels[channel].gpioPin = GPIO_IDR_IDR6;
					break;
				case 2:
					this->channels[channel].gpioPin = GPIO_IDR_IDR7;
					break;
				case 3:
					this->channels[channel].gpioPin = GPIO_IDR_IDR8;
					break;
				case 4:
					this->channels[channel].gpioPin = GPIO_IDR_IDR9;
					break;
			}
			AFIO->MAPR &= ~AFIO_MAPR_TIM4_REMAP;
		}
	}
//	else
//	{
//		// need to return error
//	}
	// from here is the only difference between reciver to pwm generator:
	this->channelsCreated += 1;
}
uint8_t timersHandler::getTotalChannels()
{
	return this->channelsCreated;
}
