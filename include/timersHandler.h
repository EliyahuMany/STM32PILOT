#ifndef __TIMERSHANDLER_H__
#define __TIMERSHANDLER_H__

#include "stm32f10x.h"
#include <memory>

typedef struct
{
	int32_t start;
	int32_t pulseWidth;
	GPIO_TypeDef *gpioPort;
	uint16_t gpioPin;
	TIM_TypeDef *timer;
	uint8_t timerChannel;
} Channel;

class timersHandler
{
	public:
		Channel *channels;
		//	Channel *channels; // TODO: change to private and implement getters and setters
		timersHandler(TIM_TypeDef **timers, uint8_t arrLen, uint8_t totalChannels);
		void configChannel(TIM_TypeDef *timer, uint8_t timerChannel, bool fiveVoltTolerance, uint8_t remoteChannel);
		bool tim4;
		uint8_t getTotalChannels();
	private:
		uint8_t tim2_remap;
		uint8_t channelsCreated;
};

#endif
