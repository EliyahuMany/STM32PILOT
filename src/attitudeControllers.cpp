#include "attitudeControllers.h"
#include "uart.h"
#include "math.h"

AttitudeControllers::AttitudeControllers(RemoteController *rc, FlightState *flightState)
{
	this->rc = rc;
	this->flightState = flightState;
}

void AttitudeControllers::pitchController(PIDController *pidController, PositionsAngles_s *UAVData, float desiredValue, float minDegreeAllowed, float maxDegreeAllowed, float minDeadZone, float maxDeadZone)
{
	pidController->calculateIdeal(UAVData->pitch, desiredValue);
	pidController->constrain(minDegreeAllowed, maxDegreeAllowed, minDeadZone, maxDeadZone); // the "next" pitch degree wanted to acheive the desired pitch. stored in pidResult	
	if (detectRCOutDeadZone(this->rc, rc->rcChannel.Elevator-1))
		changePidSetPoint(this->flightState, this->rc);
}

void AttitudeControllers::rollController(PIDController *pidController, PositionsAngles_s *UAVData, float desiredValue, float minDegreeAllowed, float maxDegreeAllowed, float minDeadZone, float maxDeadZone)
{
	pidController->calculateIdeal(UAVData->roll, desiredValue);
	pidController->constrain(minDegreeAllowed, maxDegreeAllowed, minDeadZone, maxDeadZone); // the "next" roll degree wanted to acheive the desired pitch. stored in pidResult	
	if (detectRCOutDeadZone(this->rc, rc->rcChannel.Aileron-1))
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
		case PitchHold:
			flightState->setPitchHoldParams(constrainf(flightState->GetPitchHoldParams().desiredPitch + mapDouble(rc->channels[rc->rcChannel.Elevator-1].pulseWidth, 1000, 2000, -45, 45) * 0.0075, -45.0, 45.0));
			break;
		case RollHold:
			flightState->setRollHoldParams(constrainf(flightState->GetRollHoldParams().desiredRoll + mapDouble(rc->channels[rc->rcChannel.Aileron-1].pulseWidth, 1000, 2000, -45, 45) * 0.0075, -45.0, 45.0));
			break;
		default:
			break;
	}
}

double AttitudeControllers::calcHeadingToCoord(double current_lat, double current_long, double target_lat, double target_long)
{
	double heading = atan2(target_long - current_long, target_lat - current_lat);
	return heading;
}

//double AttitudeControllers::calculate_heading_error(double current_lat, double current_long, double target_lat, double target_long, double current_heading)
//{
//  double target_heading = calcHeadingToCoord(current_lat, current_long, target_lat, target_long);
//  double heading_error = target_heading - current_heading;
//  // Normalize heading error to -180 to 180 degrees
//  //heading_error = fmod(heading_error + PI, 2 * PI) - PI;
//  return heading_error;
//}

/*

Change pid parameters (kp,ki,kd) implementation taken from scratch stage.
TODO: write function that gets the relevant pid and applying the next code.
			need to make sure the user sees the new pid values, and later on try to save it on eeprom so it can be used on mcu startup.

// PID values change:
					if (rc->channels[rc->rcChannel.SwB-1].pulseWidth > 1100)
					{
						if ((rc->channels[rc->rcChannel.VarA-1].pulseWidth > 1100) && (rc->channels[rc->rcChannel.VarA-1].pulseWidth < 1500))
						{
							if (((990 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 2010)))
							{
								oldKp = pidThrottle.getKp();
								pidThrottle.setKp(oldKp + mapDouble(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.01);
							}
						}
						else if ((rc->channels[rc->rcChannel.VarA-1].pulseWidth > 1510) && (rc->channels[rc->rcChannel.VarA-1].pulseWidth < 1800))
						{
							if (((990 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 2010)))
							{
								oldKi = pidThrottle.getKi();
								pidThrottle.setKi(oldKi + mapDouble(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.1);
							}
						}
						else if ((rc->channels[rc->rcChannel.VarA-1].pulseWidth > 1810) && (rc->channels[rc->rcChannel.VarA-1].pulseWidth < 2010))
						{
							if (((990 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 2010)))
							{
								oldKd = pidThrottle.getKd();
								pidThrottle.setKd(oldKd + mapDouble(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.001);
							}
						}
					}

*/
															 
