#ifndef __ATTITUDECONTROLLERS_H__
#define __ATTITUDECONTROLLERS_H__

#include "stm32f10x.h"
#include "PIDController.h"
#include "MPU6050.h"
#include "remotecontroller.h"
#include "flightState.h"
#include "utils.h"


class AttitudeControllers
{
	private:
		RemoteController *rc;
		FlightState *flightState;
	public:
		AttitudeControllers(RemoteController *rc, FlightState *flightState);
		void pitchController(PIDController *pidController, PositionsAngles_s *UAVData, float desiredValue, float minDegreeAllowed, float maxDegreeAllowed, float minDeadZone, float maxDeadZone);
		void rollController(PIDController *pidController, PositionsAngles_s *UAVData, float desiredValue, float minDegreeAllowed, float maxDegreeAllowed, float minDeadZone, float maxDeadZone);
		void altController(PIDController *pidController, PositionsAngles_s *UAVData, float desiredValue, float minDegreeAllowed, float maxDegreeAllowed, float minDeadZone, float maxDeadZone);
		void changePidSetPoint(FlightState *flightState, RemoteController *rc);
		bool detectRCOutDeadZone(RemoteController *rc, uint8_t rcChannel);
		double calcHeadingToCoord(double current_lat, double current_long, double target_lat, double target_long);
		//double calculate_heading_error(double current_lat, double current_long, double target_lat, double target_long, double current_heading);
};

#endif
