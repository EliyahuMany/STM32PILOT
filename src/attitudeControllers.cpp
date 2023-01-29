#include "attitudeControllers.h"
#include "uart.h"


AttitudeControllers::AttitudeControllers(RemoteController *rc, FlightState *flightState)
{
	this->rc = rc;
	this->flightState = flightState;
}

void AttitudeControllers::pitchController(PIDController *pidController, PositionsAngles_s *UAVData, float desiredValue, float minDegreeAllowed, float maxDegreeAllowed, float minDeadZone, float maxDeadZone)
{
	pidController->calculateIdeal(UAVData->pitch, desiredValue);
	pidController->constrain(minDegreeAllowed, maxDegreeAllowed, minDeadZone, maxDeadZone); // the "next" pitch degree wanted to acheive the desired pitch. stored in pidResult	
	if (detectRCOutDeadZone(this->rc, PITCH))
		changePidSetPoint(this->flightState, this->rc);
}

void AttitudeControllers::rollController(PIDController *pidController, PositionsAngles_s *UAVData, float desiredValue, float minDegreeAllowed, float maxDegreeAllowed, float minDeadZone, float maxDeadZone)
{
	pidController->calculateIdeal(UAVData->roll, desiredValue);
	pidController->constrain(minDegreeAllowed, maxDegreeAllowed, minDeadZone, maxDeadZone); // the "next" roll degree wanted to acheive the desired pitch. stored in pidResult	
	if (detectRCOutDeadZone(this->rc, ROLL))
		changePidSetPoint(this->flightState, this->rc);
}

void AttitudeControllers::altController(PIDController *pidController, PositionsAngles_s *UAVData, float desiredValue, float minDegreeAllowed, float maxDegreeAllowed, float minDeadZone, float maxDeadZone)
{
	pidController->calculateIdeal(UAVData->alt, desiredValue);
	printf("deb: alt %f %f %f\n", pidController->getPidResult(), desiredValue, pidController->getError());
	pidController->constrain(minDegreeAllowed, maxDegreeAllowed, minDeadZone, maxDeadZone); // the "next" pitch degree wanted to acheive the desired pitch. stored in pidResult	
	printf("deb: alt2 %f\n", UAVData->alt);
}

bool AttitudeControllers::detectRCOutDeadZone(RemoteController *rc, uint8_t rcChannel)
{
	if (((990 < rc->channels[rcChannel].pulseWidth) && (rc->channels[rcChannel].pulseWidth < 1490)) || ((1510 < rc->channels[rcChannel].pulseWidth) && (rc->channels[rcChannel].pulseWidth < 2010)))
	{
		return true;
	}
	else
		return false;
}

void AttitudeControllers::changePidSetPoint(FlightState *flightState, RemoteController *rc)
{
	switch (flightState->currentState)
	{
		case AirPlaneInit:
			break;
		case PitchHold: // change to PitchOld later - just for testing untill fixing the "state machine" in main
			flightState->setPitchHoldParams(constrainf(flightState->GetPitchHoldParams().desiredPitch + mapDouble(rc->channels[PITCH].pulseWidth, 1000, 2000, -45, 45) * 0.0075, -45.0, 45.0));
			break;
		case RollHold: // change to RollOld later - just for testing untill fixing the "state machine" in main
			flightState->setRollHoldParams(constrainf(flightState->GetRollHoldParams().desiredRoll + mapDouble(rc->channels[ROLL].pulseWidth, 1000, 2000, -45, 45) * 0.0075, -45.0, 45.0));
			break;
		default:
			break;
	}
}														 
