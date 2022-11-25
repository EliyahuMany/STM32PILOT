#include "flightState.h"

FlightState::FlightState()
{
	this->currentState = AirPlaneInit;
}

void FlightState::AttitudeHold(float desiredPitch, float desiredRoll)
{
	this->attitudeHoldParameters.desiredPitch = desiredPitch;
	this->attitudeHoldParameters.desiredRoll = desiredRoll;
}

AttitudeHoldParams FlightState::GetAttitudeHoldParams()
{
	return this->attitudeHoldParameters;
}

void FlightState::setAttitudeHoldRoll(float desiredRoll)
{
	this->attitudeHoldParameters.desiredRoll = desiredRoll;
}

void FlightState::setAttitudeHoldPitch(float desiredPitch)
{
	this->attitudeHoldParameters.desiredPitch = desiredPitch;
}
