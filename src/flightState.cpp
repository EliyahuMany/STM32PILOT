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

PitchHoldParams FlightState::GetPitchHoldParams()
{
	return this->pitchHoldParameters;
}

void FlightState::setPitchHoldParams(float desiredPitch)
{
	this->pitchHoldParameters.desiredPitch = desiredPitch;
}

RollHoldParams FlightState::GetRollHoldParams()
{
	return this->rollHoldParameters;
}

void FlightState::setRollHoldParams(float desiredRoll)
{
	this->rollHoldParameters.desiredRoll = desiredRoll;
}

void FlightState::setAttitudeHoldRoll(float desiredRoll)
{
	this->attitudeHoldParameters.desiredRoll = desiredRoll;
}

void FlightState::setAttitudeHoldPitch(float desiredPitch)
{
	this->attitudeHoldParameters.desiredPitch = desiredPitch;
}

void FlightState::AttitudeAltHold(float desiredAltitude)
{
	this->attitudeAltHoldParameters.desiredAltitude = desiredAltitude;
}

AttitudeAltHoldParams FlightState::GetAttitudeAltHoldParams()
{
	return this->attitudeAltHoldParameters;
}

