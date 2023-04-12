#ifndef __FLIGHTSTATE_H__
#define __FLIGHTSTATE_H__

#include "stm32f10x.h"

enum FlightStates { 
	AirPlaneInit,
	Disco,
	PitchHold,
	RollHold,
	AttitudeHold,
	AltHold,
	WPS
};

struct AttitudeHoldParams
{
	float desiredRoll;
	float desiredPitch;
};

struct PitchHoldParams
{
	float desiredPitch;
};

struct RollHoldParams
{
	float desiredRoll;
};

struct AttitudeAltHoldParams
{
	float desiredAltitude;
};

class FlightState
{
	private:
		AttitudeHoldParams attitudeHoldParameters;
		AttitudeAltHoldParams attitudeAltHoldParameters;
		PitchHoldParams pitchHoldParameters;
		RollHoldParams rollHoldParameters;
	public:
		FlightState();
		FlightStates currentState;
		void AttitudeHold(float desiredPitch, float desiredRoll);
		AttitudeHoldParams GetAttitudeHoldParams();
		void AttitudeAltHold(float desiredAltitude);
		AttitudeAltHoldParams GetAttitudeAltHoldParams();
		void setAltHoldParams(float desiredAltitude);
		PitchHoldParams GetPitchHoldParams();
		void setPitchHoldParams(float desiredPitch);
		RollHoldParams GetRollHoldParams();
		void setRollHoldParams(float desiredRoll);
		void setAttitudeHoldRoll(float desiredRoll);
		void setAttitudeHoldPitch(float desiredPitch);
		
};
#endif
