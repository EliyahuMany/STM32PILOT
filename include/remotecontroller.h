#ifndef __REMOTECONTROLLER_H__
#define __REMOTECONTROLLER_H__

#include "stm32f10x.h"
#include "timersHandler.h"

#define ROLL			0
#define PITCH			1
#define THROTTLE	2
#define YAW				3
#define VarAa			4
#define VarBb			5

const uint8_t CCR_BEGIN_ADDRESS = 0x34;

const uint16_t AFIO_EXTICR[4] = {((uint16_t)0x000F), ((uint16_t)0x00F0), ((uint16_t)0x0F00), ((uint16_t)0xF000)};

struct RemoteControlChannels
{
	enum Channel {
		Aileron=1,
		Elevator=2,
		Throttle=3,
		Rudder=4,
		VarA=5,
		VarB=6,
		SwA=7,
		SwB=8,
		SwC=9,
		SwD=10
	};
};
	
class RemoteController: public timersHandler
{
	public:
		RemoteController(TIM_TypeDef **timers, uint8_t arrLen, uint8_t totalChannels):timersHandler(timers, arrLen, totalChannels){};
		void enableTimerChannel(uint8_t channel);
		void chan_counter(uint8_t i);
		void ppm_counter(uint8_t i);
		void pp();
		void RCInit();
		int32_t getChannelPWM(uint8_t channel);
		RemoteControlChannels rcChannel;
		
};

#endif
