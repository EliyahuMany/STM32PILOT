#ifndef __PWMGENERATOR_H__
#define __PWMGENERATOR_H__

#include "stm32f10x.h"
#include "timersHandler.h"
#include "remoteController.h"
#include "utils.h"

struct PwmChannel
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

class pwmGenerator: public timersHandler
{
	public:
		pwmGenerator(TIM_TypeDef **timers, uint8_t arrLen, uint8_t totalChannels):timersHandler(timers, arrLen, totalChannels){};
		void enableTimerChannelPWM(uint8_t globalChannel);
		void sendPWM(uint8_t channel, int32_t pwmValue);
		void Disco(RemoteController* rc);
		int constrain(double pidResult);
		PwmChannel pwmChannel;
};

#endif
