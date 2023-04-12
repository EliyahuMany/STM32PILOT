#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "stm32f10x.h"
#include "i2c.h"
#include <stdlib.h>
#include "utils.h"
#include "MPU6050.h"
#include "uart.h"
#include "delay.h"
#include "remotecontroller.h"
#include "PIDController.h"
#include "SysTickHandler.h"
#include "pwmGenerator.h"
#include "flightState.h"
#include "attitudeControllers.h"
#include "Parser.h"

#define SIMULATION_MODE true
#define SIMULATION_MSG_LEN 5+(11*8)
#define LOG true

void checkForModeChange(RemoteController* rc, FlightState *flightState);

extern volatile uint32_t loop_timer,last;
extern const uint8_t anglesRoutineTimems;
extern const uint8_t servoLoopTimeMs;
extern const uint8_t logLoopTimeMs;
extern bool backFromDiscoFlag;
extern bool pitchAttitudeFirst, rollAttitudeFirst, altAttitudeFirst, throttleAttitudeFirst;
extern bool resetPidError;

extern float turn_dir;
extern Point point1, point2, point3, point4;
extern Point points[4];
extern int cur_point;
extern bool dir_choose_flag;

extern PIDController pidRoll;
extern PIDController pidPitch;
extern PIDController pidAlt;
extern PIDController pidThrottle;
extern PIDController pidHeading;


#endif
