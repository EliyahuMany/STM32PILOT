#include "config.h"


volatile uint32_t loop_timer,last = 0;
volatile bool press,int_flag = false;
const uint8_t anglesRoutineTimems = (SIMULATION_MODE) ? 13 : 4; // higher time while using simulation - first byte from python code takes long time (need to fix)
const uint8_t servoLoopTimeMs = (SIMULATION_MODE) ? anglesRoutineTimems*2 : anglesRoutineTimems*5; // when not in simulation - needs to be 20[mS] (equal to 50HZ)
const uint8_t logLoopTimeMs = anglesRoutineTimems*6;
bool backFromDiscoFlag = true;
bool pitchAttitudeFirst, rollAttitudeFirst, altAttitudeFirst, throttleAttitudeFirst = true;
bool resetPidError = true;


float turn_dir = 1.0;
Point point1 = {32.018535,34.898727};
Point point2 = {32.009635,34.894916};
Point point3 = {32.014438,34.884695};
Point point4 = {32.020624,34.889073};

Point points[4] = {point1, point2, point3, point4};
int cur_point = 0;
bool dir_choose_flag = false;

PIDController pidRoll = PIDController(anglesRoutineTimems/1000.0, 0.98, 6.8, 0.0003);
PIDController pidPitch = PIDController(anglesRoutineTimems/1000.0, 7.0, 80.0, 0.0);
PIDController pidAlt = PIDController(anglesRoutineTimems/1000.0, 1.0, 0.1, 0.01);
PIDController pidThrottle = PIDController(anglesRoutineTimems/1000.0, 2.4, 5.1, 0.04);
PIDController pidHeading = PIDController(anglesRoutineTimems/1000.0, 5.8, 0.7, 0.0);

void checkForModeChange(RemoteController* rc, FlightState *flightState)
{
	if (rc->channels[rc->rcChannel.SwA-1].pulseWidth < 1400)
	{
		flightState->currentState = Disco;
	}
	else if (rc->channels[rc->rcChannel.SwC-1].pulseWidth < 1100)
	{
		if (rc->channels[rc->rcChannel.VarB-1].pulseWidth < 1100)
		{
			if (flightState->currentState == Disco)
			{
				flightState->setRollHoldParams(0.0);
				backFromDiscoFlag = false;
			}
			flightState->currentState = RollHold;
		}
		else if ((rc->channels[rc->rcChannel.VarB-1].pulseWidth > 1100) && (rc->channels[rc->rcChannel.VarB-1].pulseWidth < 1500))
		{
			if (flightState->currentState == Disco)
			{
				flightState->setPitchHoldParams(3.0);
				backFromDiscoFlag = false;
			}
			flightState->currentState = PitchHold;
		}
		else if ((rc->channels[rc->rcChannel.VarB-1].pulseWidth > 1500) && (rc->channels[rc->rcChannel.VarB-1].pulseWidth < 2010))
		{
			flightState->currentState = AttitudeHold;
		}
	}
	else if ((rc->channels[rc->rcChannel.SwC-1].pulseWidth > 1400) && (rc->channels[rc->rcChannel.SwB-1].pulseWidth < 1800))
	{
		if (rc->channels[rc->rcChannel.VarB-1].pulseWidth < 1100)
		{
			flightState->currentState = WPS;
		}
		else if ((rc->channels[rc->rcChannel.VarB-1].pulseWidth > 1100) && (rc->channels[rc->rcChannel.VarB-1].pulseWidth < 1500))
		{
			flightState->currentState = WPS;
		}
		else if ((rc->channels[rc->rcChannel.VarB-1].pulseWidth > 1500) && (rc->channels[rc->rcChannel.VarB-1].pulseWidth < 2010))
		{
			flightState->currentState = WPS;
		}
	}
	else
	{
		if (flightState->currentState == Disco)
		{
			flightState->AttitudeAltHold(1000.0);
			backFromDiscoFlag = false;
		}
		flightState->currentState = AltHold;
	}
}
