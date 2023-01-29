#include "i2c.h"
#include "ds1307.h"
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

#define SIMULATION_MODE true
#define SIMULATION_MSG_LEN 5+(7*8)
#define LOG false

volatile uint32_t loop_timer,last = 0;
volatile unsigned char led_on;
volatile uint32_t count, prev, tot, now = 0 ;
volatile uint32_t count2, prev2, tot2 = 0 ;
volatile bool press,int_flag = false;
const uint8_t anglesRoutineTimems = 13; // higher time while using simulation - first byte from python code takes long time (need to fix)
const uint8_t servoLoopTimeMs = anglesRoutineTimems*2;
const uint8_t logLoopTimeMs = anglesRoutineTimems*6;
bool start = true;

double oldKp,oldKi,oldKd;

bool pitchAttitudeFirst, rollAttitudeFirst, altAttitudeFirst, throttleAttitudeFirst = true;

/**
RC Channels:

*/

union hexToFloat
{
	char c[4];
	float fval;
	int ival;
};

RemoteController *rc; // need to find a way to attach class members function into interrupts without creating global variables like this

// need to find a way to attach functions to interrupt handlers, and not in main.cpp
extern "C" void TIM4_IRQHandler(void)
{
//	rc->chan_counter(rc->rcChannel.Elevator); // get pulseWidth of Remote Control channel N
//	rc->chan_counter(rc->rcChannel.SwC);
//	rc->chan_counter(rc->rcChannel.Aileron);

	rc->ppm_counter(rc->rcChannel.Elevator); // need to send only one of the first 8 channels because using PPM
	
}
extern "C" void TIM3_IRQHandler(void)
{
	rc->chan_counter(5);
}



struct simpleLogger
{
	float throttle, throttleCmd;
	float pitch, pitchCmd, pitchDesired;
	float roll, rollCmd, rollDesired;
	float alt, altDesired;
	int mode; // 1 Roll, 2 Pitch, 3 Alt, 4 Disco
};



int main()
{
	
	
	uartHandler *huart1 = new uartHandler(USART1, 115200);
	
	
	TIM_TypeDef* pwmTimers[] = {TIM3};
	pwmGenerator *pwmHandler;
	pwmHandler = new pwmGenerator(pwmTimers, sizeof(pwmTimers)/sizeof(pwmTimers[0]), 2);
	
	pwmHandler->configChannel(TIM3, 2, false, pwmHandler->pwmChannel.Elevator); // PA7
	pwmHandler->enableTimerChannelPWM(pwmHandler->pwmChannel.Elevator);
	pwmHandler->configChannel(TIM3, 1, false, pwmHandler->pwmChannel.Aileron); // PA6
	pwmHandler->enableTimerChannelPWM(pwmHandler->pwmChannel.Aileron);

	
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	
	GPIOB->CRH &= ~GPIO_CRH_MODE13_0;
	GPIOB->CRH |= GPIO_CRH_MODE13_1;
	
	GPIOB->CRH &= ~GPIO_CRH_CNF13_0;
	GPIOB->CRH &= ~GPIO_CRH_CNF13_1;
	
	GPIOB->CRH &= ~GPIO_CRH_MODE14_0;
	GPIOB->CRH |= GPIO_CRH_MODE14_1;
	
	GPIOB->CRH &= ~GPIO_CRH_CNF14_0;
	GPIOB->CRH &= ~GPIO_CRH_CNF14_1;
	
	GPIOB->CRH &= ~GPIO_CRH_MODE15_0;
	GPIOB->CRH |= GPIO_CRH_MODE15_1;
	
	GPIOB->CRH &= ~GPIO_CRH_CNF15_0;
	GPIOB->CRH &= ~GPIO_CRH_CNF15_1;
	
	
	SysTick_Config(SystemCoreClock/1000U); // config systick to call his handler every 1ms
	
	I2C_Handler *hi2c2 = new I2C_Handler(I2C2,false,false,8000000,200000);
	
	MPU6050 mpu = MPU6050(hi2c2, 0x68);
	for (int i=0;i<10000000;i++); // small delay in order to prevent touch movement to be taken into the init process
	mpu.mpuInit();
	
	PIDController pidRoll = PIDController(anglesRoutineTimems/1000.0, 0.98, 6.8, 0.0003);
	PIDController pidPitch = PIDController(anglesRoutineTimems/1000.0, 7.0, 80.0, 0.0);
	PIDController pidPitchAlt = PIDController(anglesRoutineTimems/1000.0, 1.1, 10.0, 0.0);
	PIDController pidAlt = PIDController(anglesRoutineTimems/1000.0, 1.0, 0.1, 0.01);
	PIDController pidThrottle = PIDController(anglesRoutineTimems/1000.0, 2.4, 5.1, 0.04);
	PIDController pidHeading = PIDController(anglesRoutineTimems/1000.0, 1.0, 0.1, 0.01);
	

	TIM_TypeDef* timers[] = {TIM4};
	rc = new RemoteController(timers, sizeof(timers)/sizeof(timers[0]), 10);
	rc->RCInit();
	FlightState *flightState = new FlightState();
	AttitudeControllers attitudeController(rc, flightState);
	PositionsAngles_s UAVdata;
	simpleLogger log;
	log.mode = 4;

	flightState->currentState = Disco;
	start = false;
	
	char receivedBytes[SIMULATION_MSG_LEN];
	hexToFloat conv;
	loop_timer = GetTick();

	while (true)
	{
		if (flightState->currentState == Disco)
		{
			if ((GetTick() > loop_timer) && ((GetTick() - loop_timer) % 20 == 0))
			{
				if (log.mode != 4)
					log.mode = 4;
				if (SIMULATION_MODE)
				{
					// send command to change pitch
					printf("DP%f\n", float(mapDouble(rc->channels[rc->rcChannel.Elevator-1].pulseWidth, 1000, 2000, -1.0, 1.0)));
					printf("DR%f\n", float(mapDouble(rc->channels[rc->rcChannel.Aileron-1].pulseWidth, 1000, 2000, -1.0, 1.0)));
					printf("DT%f\n", float(mapDouble(rc->channels[rc->rcChannel.Throttle-1].pulseWidth, 1500, 2000, 0.0, 1.0)));
					UAVdata.throttle = float(mapDouble(rc->channels[rc->rcChannel.Throttle-1].pulseWidth, 1500, 2000, 0.0, 1.0));
					log.rollCmd = float(mapDouble(rc->channels[rc->rcChannel.Aileron-1].pulseWidth, 1000, 2000, -1.0, 1.0));
					log.pitchCmd = float(mapDouble(rc->channels[rc->rcChannel.Elevator-1].pulseWidth, 1000, 2000, -1.0, 1.0));
					log.throttleCmd = UAVdata.throttle;
				}
				else
				{
					pwmHandler->Disco(rc);
				}
				// check if switch changed position - change flight state
				if (rc->channels[rc->rcChannel.SwA-1].pulseWidth > 1800)
				{
					if (rc->channels[rc->rcChannel.SwB-1].pulseWidth < 1100)
					{
						flightState->currentState = RollHold;
						flightState->setRollHoldParams(0.0);
					}
					else if ((rc->channels[rc->rcChannel.SwB-1].pulseWidth > 1400) && (rc->channels[rc->rcChannel.SwB-1].pulseWidth < 1800))
					{
						flightState->currentState = PitchHold;
						flightState->setPitchHoldParams(10.0);
					}
					else
					{
						flightState->currentState = AltHold;
						flightState->AttitudeAltHold(1000.0);
					}
					start = false;
				}
				while(((GetTick() - loop_timer) % 20 == 0));
				//loop_timer = GetTick();
			}
			if ((GetTick() > loop_timer) && ((GetTick() - loop_timer) % 25 == 0) && LOG == true)
			{
				printf("log:m%d\n", log.mode);
				printf("log:ps%f\n", log.pitch);
				printf("log:pc%f\n", log.pitchCmd);
				printf("log:pd%f\n", log.pitchDesired);
				printf("log:rs%f\n", log.roll);
				printf("log:rc%f\n", log.rollCmd);
				printf("log:rd%f\n", log.rollDesired);
				printf("log:ts%f\n", log.throttle);
				printf("log:tc%f\n", log.throttleCmd);
				printf("log:as%f\n", log.alt);
				printf("log:ad%f\n", log.altDesired);
				while((GetTick() - loop_timer) % 25 == 0);
				loop_timer = GetTick();
			}
		}
		else if (flightState->currentState == AttitudeHold || flightState->currentState == PitchHold || flightState->currentState == RollHold || flightState->currentState == AltHold)
		{
			if (!start)
			{
				loop_timer = GetTick();
				start = true;
				if (!pitchAttitudeFirst)
						pitchAttitudeFirst = true;
				if (!rollAttitudeFirst)
							rollAttitudeFirst = true;
				if (!altAttitudeFirst)
							altAttitudeFirst = true;
				if (!throttleAttitudeFirst)
							throttleAttitudeFirst = true;
			}
			if ((GetTick() >= loop_timer) && ((GetTick() - loop_timer) % servoLoopTimeMs == 0))
			{
				GPIOB->ODR ^= GPIO_ODR_ODR14;
				if (flightState->currentState == PitchHold)
				{
					if (pitchAttitudeFirst)
					{
						pidPitch.resetError();
						log.mode = 2;
					}
					attitudeController.pitchController(&pidPitch, &UAVdata, flightState->GetPitchHoldParams().desiredPitch, -25.0, 25.0, 0, 0);
					if (SIMULATION_MODE)
					{
						// send command to change pitch*
						printf("PC%f\n", float(mapDouble(pidPitch.getPidResult(), -45.0, 45.0, -1.0, 1.0)*-1.0)); // TODO: check what happend when in_min and in_max equals (when i set them both to -25.0 the simulator crashed)
						log.pitchCmd = float(mapDouble(pidPitch.getPidResult(), -45.0, 45.0, -1.0, 1.0)*-1.0);
						log.pitchDesired = flightState->GetPitchHoldParams().desiredPitch;
						printf("DR%f\n", float(mapDouble(rc->channels[rc->rcChannel.Aileron-1].pulseWidth, 1000, 2000, -1.0, 1.0)));
					}
				}
				else if (flightState->currentState == RollHold)
				{
					if (rollAttitudeFirst)
					{
						pidRoll.resetError();
						log.mode = 1;
					}
					attitudeController.rollController(&pidRoll, &UAVdata, flightState->GetRollHoldParams().desiredRoll, -30.0, 30.0, 0, 0);

					if (SIMULATION_MODE)
					{
						// send command to change pitch*
						printf("RC%f\n", float(mapDouble(pidRoll.getPidResult(), -45.0, 45.0, -1.0, 1.0)*-1.0)); // TODO: check what happend when in_min and in_max equals (when i set them both to -25.0 the simulator crashed)
						log.rollCmd = float(mapDouble(pidRoll.getPidResult(), -45.0, 45.0, -1.0, 1.0)*-1.0);
						log.rollDesired = flightState->GetRollHoldParams().desiredRoll;
						printf("DP%f\n", float(mapDouble(rc->channels[rc->rcChannel.Elevator-1].pulseWidth, 1000, 2000, -1.0, 1.0))); // TODO: add deg parmeter so only one command will change the servos/simulation.
					}
					else
					{
						pwmHandler->sendPWM(rc->rcChannel.Aileron, pwmHandler->constrain(pidRoll.getPidResult()));
						pwmHandler->sendPWM(rc->rcChannel.Elevator, rc->getChannelPWM(rc->rcChannel.Elevator));
					}
					// TODO: add pid change function that will fit all rcChannels.
				}
				else if (flightState->currentState == AttitudeHold)
				{
					if (rollAttitudeFirst)
						pidRoll.resetError();
					if (pitchAttitudeFirst)
						pidPitch.resetError();
					attitudeController.pitchController(&pidPitch, &UAVdata, flightState->GetPitchHoldParams().desiredPitch, -25.0, 25.0, 0, 0);
					attitudeController.rollController(&pidRoll, &UAVdata, flightState->GetRollHoldParams().desiredRoll, -30.0, 30.0, 0, 0);
					if (SIMULATION_MODE)
					{
						// send command to change pitch*
						printf("RC%f\n", float(mapDouble(pidRoll.getPidResult(), -45.0, 45.0, -1.0, 1.0))*-1.0);
						printf("PC%f\n", float(mapDouble(pidPitch.getPidResult(), -45.0, 45.0, -1.0, 1.0))*-1.0);
						printf("DT%f\n", float(mapDouble(rc->channels[rc->rcChannel.Throttle-1].pulseWidth, 1500, 2000, 0.0, 1.0))); 
						UAVdata.throttle = float(mapDouble(rc->channels[rc->rcChannel.Throttle-1].pulseWidth, 1500, 2000, 0.0, 1.0));
					}
					else
					{
						if (pitchAttitudeFirst)
						{
							pitchAttitudeFirst = false;
							pidPitch.resetError();
						}
						pwmHandler->sendPWM(rc->rcChannel.Elevator, pwmHandler->constrain(pidPitch.getPidResult()));
						pwmHandler->sendPWM(rc->rcChannel.Aileron, pwmHandler->constrain(pidRoll.getPidResult()));

					}
				}
				else if (flightState->currentState == AltHold)
				{
					if (altAttitudeFirst)
					{
						pidAlt.resetError();
						pidThrottle.resetError();
						altAttitudeFirst = false;
						log.mode = 3;
					}
					pidAlt.calculateIdeal(UAVdata.alt, flightState->GetAttitudeAltHoldParams().desiredAltitude);
					pidThrottle.calculateIdeal(UAVdata.throttle, pidAlt.getPidResult());
					pidThrottle.constrain(-10.0, 10.0, -3.0, 3.0);
					log.altDesired = flightState->GetAttitudeAltHoldParams().desiredAltitude;
					if (SIMULATION_MODE)
					{
						// send command to change throttle*
						printf("TC%f\n", float(mapDouble(pidThrottle.getPidResult(), -10.0, 10.0, 0.8, 1.0)));
						log.throttleCmd = float(mapDouble(pidThrottle.getPidResult(), -10.0, 10.0, 0.8, 1.0));
						// control pitch/roll via RC
						printf("DP%f\n", float(mapDouble(rc->channels[rc->rcChannel.Elevator-1].pulseWidth, 1000, 2000, -1.0, 1.0)));
						printf("DR%f\n", float(mapDouble(rc->channels[rc->rcChannel.Aileron-1].pulseWidth, 1000, 2000, -1.0, 1.0)));
					}
					else
					{
						pwmHandler->sendPWM(rc->rcChannel.Throttle, pwmHandler->constrain(pidThrottle.getPidResult()));
						pwmHandler->sendPWM(rc->rcChannel.Aileron, rc->getChannelPWM(rc->rcChannel.Aileron));
						pwmHandler->sendPWM(rc->rcChannel.Elevator, rc->getChannelPWM(rc->rcChannel.Elevator));
					}
					
					// PID values change:
					if (rc->channels[rc->rcChannel.SwB-1].pulseWidth > 1100)
					{
						if ((rc->channels[rc->rcChannel.VarA-1].pulseWidth > 1100) && (rc->channels[rc->rcChannel.VarA-1].pulseWidth < 1500))
						{
							if (((990 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 2010)))
							{
								oldKp = pidThrottle.getKp();
								//printf("%f\n",map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.0001);
								pidThrottle.setKp(oldKp + mapDouble(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.01);
								printf("P: %f\n",pidThrottle.getKp());
							}
						}
						else if ((rc->channels[rc->rcChannel.VarA-1].pulseWidth > 1510) && (rc->channels[rc->rcChannel.VarA-1].pulseWidth < 1800))
						{
							if (((990 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 2010)))
							{
								oldKi = pidThrottle.getKi();
								//printf("%f\n",map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.0001);
								pidThrottle.setKi(oldKi + mapDouble(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.1);
								printf("I: %f\n",pidThrottle.getKi());
							}
						}
						else if ((rc->channels[rc->rcChannel.VarA-1].pulseWidth > 1810) && (rc->channels[rc->rcChannel.VarA-1].pulseWidth < 2010))
						{
							if (((990 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 2010)))
							{
								oldKd = pidThrottle.getKd();
								//printf("%f\n",map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.0001);
								pidThrottle.setKd(oldKd + mapDouble(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.001);
								printf("D: %f\n",pidThrottle.getKd());
							}
						}
					}
				}
				if (rc->channels[rc->rcChannel.SwA-1].pulseWidth < 1400)
				{
					flightState->currentState = Disco;
					if (log.mode != 4)
						log.mode = 4;
				}
				else if (rc->channels[rc->rcChannel.SwB-1].pulseWidth < 1100)
				{
					flightState->currentState = RollHold;
				}
				else if ((rc->channels[rc->rcChannel.SwB-1].pulseWidth > 1400) && (rc->channels[rc->rcChannel.SwB-1].pulseWidth < 1800))
				{
					flightState->currentState = PitchHold;
				}
				else
				{
					flightState->currentState = AltHold;
				}
				
				
				while((GetTick() > loop_timer) && ((GetTick() - loop_timer) % servoLoopTimeMs != 0));
				GPIOB->ODR ^= GPIO_ODR_ODR14;
				loop_timer = GetTick();
			}
			if ((GetTick() >= loop_timer) && ((GetTick() - loop_timer) % anglesRoutineTimems == 0))
			{
				GPIOB->ODR ^= GPIO_ODR_ODR13;
				if (SIMULATION_MODE)
				{
					/* TODO:
						when sending data over uart, it takes for the python code time to proccess it and sending back.
						while in simulation mode - the loops times will be bigger then the loops time using hardware
					*/
					// read 6dof simulation data and store
					// request data
					huart1->write('S');
					huart1->write('e');
					huart1->write('n');
					huart1->write('d');
					huart1->write('P');
					huart1->write('\n');
					huart1->Scanf(receivedBytes, SIMULATION_MSG_LEN); // must know x-plane return msg size
					//GPIOB->ODR &= ~GPIO_ODR_ODR13;
					//printf("deb: %c%c%c%c%c\n",receivedBytes[0],receivedBytes[1],receivedBytes[2],receivedBytes[3],receivedBytes[4]);
					if ((receivedBytes[0] == 'R') && (receivedBytes[1] == 'R') && (receivedBytes[2] == 'E') && (receivedBytes[3] == 'F') && (receivedBytes[4] == ','))
					{
						// TODO: add crc check of the data4
						uint8_t byteCounter = 1;
						float parsedValue = -999.0;
						bool indexFlag = true;
						uint8_t packetIndex = 0;
						for (int i=5;i<SIMULATION_MSG_LEN;i++)
						{
							if (byteCounter == 4) // after reading 4 bytes
							{
								conv.c[byteCounter-1] = receivedBytes[i];
								parsedValue = conv.fval;
								if (indexFlag)
									packetIndex = conv.ival;
								else
								{
									switch (packetIndex)
									{
										case 0:
											// Pitch deg
											if (parsedValue >= -90 && parsedValue <= 90)
											{
												UAVdata.pitch = parsedValue;
												log.pitch = UAVdata.pitch;
												//printf("deb: Pitch %f\n",parsedValue);
											}
											break;
										case 1:
											// Roll deg
											if (parsedValue >= -90 && parsedValue <= 90)
											{
												UAVdata.roll = parsedValue;
												log.roll = UAVdata.roll;
											}
											//printf("deb: R%f\n",parsedValue);
											break;
										case 2:
											// Alt msl
											if (parsedValue >= -500 && parsedValue <= 20000)
											{	
												UAVdata.alt = parsedValue;
												log.alt = UAVdata.alt;
												//printf("deb: A%f\n",parsedValue);
											}
											break;
										case 3:
											// throttle
											if (parsedValue >= -0.0 && parsedValue <= 1.0)
											{	
												UAVdata.throttle = parsedValue;
												log.throttle = UAVdata.throttle;
												//printf("deb: A%f\n",parsedValue);
											}
											break;
										case 4:
											// heading
											if (parsedValue >= 0.0 && parsedValue <= 360.0)
											{	
												UAVdata.yaw = parsedValue;
												//printf("deb: A%f\n",parsedValue);
											}
											break;
										case 5:
											// latitude
											if (parsedValue >= -90.0 && parsedValue <= 90.0)
											{	
												UAVdata.gps.lat = parsedValue;
											}
											break;
										case 6:
											// longitude
											if (parsedValue >= -180.0 && parsedValue <= 180.0)
											{	
												UAVdata.gps.lon = parsedValue;
											}
											break;
									}
								}
								byteCounter = 1;
								indexFlag = !indexFlag;
							}
							else
							{
								conv.c[byteCounter-1] = receivedBytes[i]; // Little Endian
								byteCounter += 1;
							}
						}
					}
					huart1->write('A');
					huart1->write('c');
					huart1->write('k');
					huart1->write('\n');
				}
				else
				{
					mpu.anglesAccel();
					mpu.anglesGyro(anglesRoutineTimems);
					mpu.combinedAngles();
					// insert mpu angles into PositionsAngles_s struct
					UAVdata.pitch = mpu.CombinedAngles_s.pitch;
					UAVdata.yaw = mpu.CombinedAngles_s.yaw;
					UAVdata.roll = mpu.CombinedAngles_s.roll;
					log.pitch = UAVdata.pitch;
					log.roll = UAVdata.roll;
				}
				while((GetTick() > loop_timer) && ((GetTick() - loop_timer) % anglesRoutineTimems != 0));
				GPIOB->ODR ^= GPIO_ODR_ODR13;
				// because this is the fastast loop, dont "reset" loop_timer or else it will keep looping here
			}
			else if ((GetTick() >= loop_timer) && ((GetTick() - loop_timer) % logLoopTimeMs == 0) && LOG == true)
			{
				GPIOB->ODR ^= GPIO_ODR_ODR15;
				printf("log:m%d\n", log.mode);
				printf("log:ps%f\n", log.pitch);
				printf("log:pc%f\n", log.pitchCmd);
				printf("log:pd%f\n", log.pitchDesired);
				printf("log:rs%f\n", log.roll);
				printf("log:rc%f\n", log.rollCmd);
				printf("log:rd%f\n", log.rollDesired);
				printf("log:ts%f\n", log.throttle);
				printf("log:tc%f\n", log.throttleCmd);
				printf("log:as%f\n", log.alt);
				printf("log:ad%f\n", log.altDesired);
				while((GetTick() > loop_timer) && ((GetTick() - loop_timer) % logLoopTimeMs != 0));
				loop_timer = GetTick();
			}
		}
	}
	return 0;
}
