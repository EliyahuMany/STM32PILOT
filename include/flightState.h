#ifndef __FLIGHTSTATE_H__
#define __FLIGHTSTATE_H__

#include "stm32f10x.h"

enum FlightStates { 
	AirPlaneInit,
	Disco,
	Rates,
	AttitudeHold
};

struct AttitudeHoldParams
{
	float desiredRoll;
	float desiredPitch;
	
};


class FlightState
{
	private:
		AttitudeHoldParams attitudeHoldParameters;
	public:
		FlightState();
		FlightStates currentState;
		FlightStates lastState;
		void AttitudeHold(float desiredPitch, float desiredRoll);
		AttitudeHoldParams GetAttitudeHoldParams();
		void setAttitudeHoldRoll(float desiredRoll);
		void setAttitudeHoldPitch(float desiredPitch);
};
#endif
