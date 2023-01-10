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
#define SIMULATION_MSG_LEN 37

//volatile uint32_t msTicks, _tick,loop_timer = 0 ;
volatile uint32_t loop_timer,last = 0;
volatile unsigned char led_on;
volatile uint32_t count, prev, tot, now = 0 ;
volatile uint32_t count2, prev2, tot2 = 0 ;
volatile bool press,int_flag = false;
const uint8_t anglesRoutineTimems = 4;
const uint8_t servoLoopTimeMs = 4*5-1;
int loopCounter = 0;
int wtf = 0;
bool start = false;

double oldKp,oldKi,oldKd;

float simFloatVar;

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
	
	
	SysTick_Config(SystemCoreClock/1000U); // config systick to call his handler every 1ms
	
	I2C_Handler *hi2c2 = new I2C_Handler(I2C2,false,false,8000000,200000);
	
	MPU6050 mpu = MPU6050(hi2c2, 0x68);
	for (int i=0;i<10000000;i++); // small delay for preventing touch movement
	mpu.mpuInit();
	
	PIDController pidRoll = PIDController(anglesRoutineTimems/1000.0, 0.98, 6.8, 0.0003);
	PIDController pidPitch = PIDController(anglesRoutineTimems/1000.0, 7.0, 80.0, 0.0);
	PIDController pidPitchAlt = PIDController(anglesRoutineTimems/1000.0, 1.1, 10.0, 0.0);
	PIDController pidAlt = PIDController(anglesRoutineTimems/1000.0, 1.0, 0.1, 0.01);
	PIDController pidThrottle = PIDController(anglesRoutineTimems/1000.0, 2.4, 5.1, 0.04);
	

	TIM_TypeDef* timers[] = {TIM4};
	rc = new RemoteController(timers, sizeof(timers)/sizeof(timers[0]), 10);
	rc->RCInit();
	FlightState *flightState = new FlightState();
	AttitudeControllers attitudeController(rc, flightState);
	PositionsAngles_s UAVdata;

	flightState->currentState = Disco;
	start = false;
	
	char testBytes[61];
	hexToFloat conv;
	
	while (true)
	{
		if (flightState->currentState == Disco)
		{
			//loop_timer = GetTick();
			// no need to check mpu and other things, just pass the user receiver values
			if (SIMULATION_MODE)
			{
				// send command to change pitch
				printf("DP%f\n", float(mapDouble(rc->channels[rc->rcChannel.Elevator-1].pulseWidth, 1000, 2000, -1.0, 1.0)));
				printf("DR%f\n", float(mapDouble(rc->channels[rc->rcChannel.Aileron-1].pulseWidth, 1000, 2000, -1.0, 1.0)));
				printf("DT%f\n", float(mapDouble(rc->channels[rc->rcChannel.Throttle-1].pulseWidth, 1500, 2000, 0.0, 1.0)));
				UAVdata.throttle = float(mapDouble(rc->channels[rc->rcChannel.Throttle-1].pulseWidth, 1500, 2000, 0.0, 1.0));
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
					flightState->setPitchHoldParams(20.0);
				}
				else
				{
					flightState->currentState = AltHold;
					flightState->AttitudeAltHold(1000.0);
				}
				start = false;
			}
			while(GetTick() - loop_timer < 20);
			loop_timer = GetTick();
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
			if ((GetTick() > loop_timer) && ((GetTick() - loop_timer) % servoLoopTimeMs == 0))
			{
				if (flightState->currentState == PitchHold)
				{
					if (pitchAttitudeFirst)
						pidPitch.resetError();
					GPIOB->ODR ^= GPIO_ODR_ODR14;
					attitudeController.pitchController(&pidPitch, &UAVdata, flightState->GetPitchHoldParams().desiredPitch, -25.0, 25.0, 0, 0);
					if (SIMULATION_MODE)
					{
						// send command to change pitch*
						printf("PC%f\n", float(mapDouble(pidPitch.getPidResult(), -45.0, 45.0, -1.0, 1.0)*-1.0)); // TODO: check what happend when in_min and in_max equals (when i set them both to -25.0 the simulator crashed)
						printf("DR%f\n", float(mapDouble(rc->channels[rc->rcChannel.Aileron-1].pulseWidth, 1000, 2000, -1.0, 1.0)));
					}
				}
				else if (flightState->currentState == RollHold)
				{
					if (rollAttitudeFirst)
						pidRoll.resetError();
					GPIOB->ODR ^= GPIO_ODR_ODR14;
					attitudeController.rollController(&pidRoll, &UAVdata, flightState->GetRollHoldParams().desiredRoll, -30.0, 30.0, 0, 0);
					if (SIMULATION_MODE)
					{
						// send command to change pitch*
						printf("RC%f\n", float(mapDouble(pidRoll.getPidResult(), -45.0, 45.0, -1.0, 1.0)*-1.0)); // TODO: check what happend when in_min and in_max equals (when i set them both to -25.0 the simulator crashed)
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
					GPIOB->ODR ^= GPIO_ODR_ODR14;
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
					}
					GPIOB->ODR ^= GPIO_ODR_ODR14;
					pidAlt.calculateIdeal(UAVdata.alt, flightState->GetAttitudeAltHoldParams().desiredAltitude);
					pidThrottle.calculateIdeal(UAVdata.throttle, pidAlt.getPidResult());
					pidThrottle.constrain(-10.0, 10.0, -3.0, 3.0);
					if (SIMULATION_MODE)
					{
						// send command to change throttle*
						printf("TC%f\n", float(mapDouble(pidThrottle.getPidResult(), -10.0, 10.0, 0.8, 1.0)));
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
				
				//printf("%.1f\t%.1f\t%.1f\n", mpu.CombinedAngles_s.pitch, mpu.CombinedAngles_s.roll, mpu.CombinedAngles_s.yaw);
				
				while((GetTick() > loop_timer) && ((GetTick() - loop_timer) % servoLoopTimeMs == 0));
				loop_timer = GetTick();
			}
			if (((GetTick() - loop_timer) % anglesRoutineTimems == 0))
			{
				GPIOB->ODR |= GPIO_ODR_ODR13;
				if (SIMULATION_MODE)
				{
					// read 6dof simulation data and store
					// request data
					huart1->write('S');
					huart1->write('e');
					huart1->write('n');
					huart1->write('d');
					huart1->write('P');
					huart1->write('\n');
					huart1->Scanf(testBytes, SIMULATION_MSG_LEN); // must know x-plane return msg size
					GPIOB->ODR &= ~GPIO_ODR_ODR13;
					//printf("deb: %c%c%c%c%c\n",testBytes[0],testBytes[1],testBytes[2],testBytes[3],testBytes[4]);
					if ((testBytes[0] == 'R') && (testBytes[1] == 'R') && (testBytes[2] == 'E') && (testBytes[3] == 'F') && (testBytes[4] == ','))
					{
						// TODO: add crc check of the data4
						uint8_t byteCounter = 1;
						float parsedValue = 0.0;
						bool indexFlag = true;
						uint8_t packetIndex = 0;
						for (int i=5;i<SIMULATION_MSG_LEN;i++)
						{
							if (byteCounter == 4) // after reading 4 bytes
							{
								conv.c[byteCounter-1] = testBytes[i];
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
												//printf("deb: P%f\n",parsedValue);
											}
											break;
										case 1:
											// Roll deg
											if (parsedValue >= -90 && parsedValue <= 90)
												UAVdata.roll = parsedValue;
											//printf("deb: R%f\n",parsedValue);
											break;
										case 2:
											// Alt msl
											if (parsedValue >= -500 && parsedValue <= 20000)
											{	
												UAVdata.alt = parsedValue;
												//printf("deb: A%f\n",parsedValue);
											}
											break;
										case 3:
											// throttle
											if (parsedValue >= -0.0 && parsedValue <= 1.0)
											{	
												UAVdata.throttle = parsedValue;
												//printf("deb: A%f\n",parsedValue);
											}
									}
								}
								byteCounter = 1;
								indexFlag = !indexFlag;
							}
							else
							{
								conv.c[byteCounter-1] = testBytes[i]; // Little Endian
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
				}
				while((GetTick() - loop_timer) % anglesRoutineTimems == 0);
			}
		}
	}
	return 0;
}
