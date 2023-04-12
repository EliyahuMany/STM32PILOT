#include "config.h"

RemoteController *rc; // need to find a way to attach class members function into interrupts without creating global variables like this (this one created because of the interrupt handler)
extern "C" void TIM4_IRQHandler(void)// need to find a way to attach functions to interrupt handlers, and not in main.cpp
{
//	when not using ppm:
//	rc->chan_counter(rc->rcChannel.Elevator); // get pulseWidth of Remote Control channel N
//	rc->chan_counter(rc->rcChannel.SwC);
//	rc->chan_counter(rc->rcChannel.Aileron);
	rc->ppm_counter(rc->rcChannel.Elevator); // need to send only one of the first 8 channels because using PPM	
}

int main()
{
	uartHandler *huart1 = new uartHandler(USART1, 115200);
	// control servos of the airplane (when not on simulation)
	TIM_TypeDef* pwmTimers[] = {TIM3};
	pwmGenerator *pwmHandler;
	pwmHandler = new pwmGenerator(pwmTimers, sizeof(pwmTimers)/sizeof(pwmTimers[0]), 2);
	
	pwmHandler->configChannel(TIM3, 2, false, pwmHandler->pwmChannel.Elevator); // PA7
	pwmHandler->enableTimerChannelPWM(pwmHandler->pwmChannel.Elevator);
	pwmHandler->configChannel(TIM3, 1, false, pwmHandler->pwmChannel.Aileron); // PA6
	pwmHandler->enableTimerChannelPWM(pwmHandler->pwmChannel.Aileron);	
	
	SysTick_Config(SystemCoreClock/1000U); // config systick to call his handler every 1ms

	I2C_Handler *hi2c2 = new I2C_Handler(I2C2,false,false,8000000,200000);
	
	MPU6050 mpu = MPU6050(hi2c2, 0x68);
	for (int i=0;i<10000000;i++); // small delay in order to prevent touch movement to be taken into the init process
	mpu.mpuInit();
	
	
	// TODO: add regular RC setup option (without PPM)
	TIM_TypeDef* timers[] = {TIM4};
	rc = new RemoteController(timers, sizeof(timers)/sizeof(timers[0]), 10);
	rc->RCInit();
	FlightState *flightState = new FlightState();
	AttitudeControllers attitudeController(rc, flightState);
	PositionsAngles_s UAVdata;
	simpleLogger log;
	log.mode = 4;
	flightState->currentState = Disco;
	backFromDiscoFlag = false;
	char receivedBytes[SIMULATION_MSG_LEN];
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
					printf("DP%f\n", float(mapDouble(rc->channels[rc->rcChannel.Elevator-1].pulseWidth, 1000, 2000, -1.0, 1.0)));
					printf("DR%f\n", float(mapDouble(rc->channels[rc->rcChannel.Aileron-1].pulseWidth, 1000, 2000, -1.0, 1.0)));
					printf("DH%f\n", float(mapDouble(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -1.0, 1.0)));
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
				checkForModeChange(rc, flightState);
				while(((GetTick() - loop_timer) % 20 == 0));
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
				printf("log:hs%f\n", log.heading);
				printf("log:dw%f\n", log.distanceToWP);
				printf("log:bw%f\n", log.bearingToWP);
				while((GetTick() - loop_timer) % 25 == 0);
				loop_timer = GetTick();
			}
		}
		else if (flightState->currentState == AttitudeHold || flightState->currentState == WPS || flightState->currentState == RollHold || flightState->currentState == AltHold || flightState->currentState == PitchHold)
		{
			if (!backFromDiscoFlag)
			{
				loop_timer = GetTick();
				backFromDiscoFlag = true;
				resetPidError = true;
			}
			if ((GetTick() >= loop_timer) && ((GetTick() - loop_timer) % servoLoopTimeMs == 0))
			{
				if (flightState->currentState == PitchHold)
				{
					if (resetPidError)
					{
						resetPidError = false;
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
				else if (flightState->currentState == WPS)
				{
					if (resetPidError)
					{
						resetPidError = false;
						pidPitch.resetError();
						pidRoll.resetError();
						log.mode = 5;
					}
					Point current = points[cur_point];
					Point pos = {UAVdata.gps.lat, UAVdata.gps.lon};
					// calculate distance to the wp
					double distance = haversineDistance(pos, current);
					log.distanceToWP = distance;
					if (distance <= 50.0)
					{
						dir_choose_flag = false;
						cur_point += 1;
						if (cur_point == 4)
							cur_point = 0;
					}
					
					double bear = bearing(pos.lat, pos.lon, current.lat, current.lon);
					double rel_bear = getRelativeBearing(UAVdata.yaw, rad2circleDeg(bear));
					log.bearingToWP = rad2deg(rel_bear);
					if (!dir_choose_flag)
					{
						dir_choose_flag = true;
						double heading_diff = fmod(rad2circleDeg(bear) - UAVdata.yaw+360, 360);
						
						if (heading_diff < 180)
							turn_dir = -1.0;
						else
							turn_dir = 1.0;
					}
					double error = unwrap(bear) - unwrap(deg2rad(UAVdata.yaw));
					pidHeading.calculate(1.0, 1.0, rad2deg(error));
					pidHeading.constrain(-180.0, 180.0, 0.0, 0.0);
					double pcmd = (pidHeading.getPidResult() - UAVdata.roll)*0.23;
					pcmd = pcmd - UAVdata.rollRate*0.0014;
					pcmd = constrainf(pcmd, -30.0, 30.0);
					pidRoll.calculateIdeal(UAVdata.roll, pcmd);
					pidRoll.constrain(-30.0, 30.0, 0.0, 0.0);
					log.rollCmd = float(mapDouble(pidRoll.getPidResult(), -45.0, 45.0, -1.0, 1.0)*-1.0);
					attitudeController.pitchController(&pidPitch, &UAVdata, 1.0, -25.0, 25.0, 0, 0);
					log.pitchDesired = 1.0;
					if (SIMULATION_MODE)
					{
						// send command to change pitch*
						printf("RC%f\n", float(mapDouble(pidRoll.getPidResult(), -45.0, 45.0, -1.0, 1.0)*-1.0));
						printf("PC%f\n", float(mapDouble(pidPitch.getPidResult(), -45.0, 45.0, -1.0, 1.0)*-1.0));
						printf("DH%f\n", float(mapDouble(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -1.0, 1.0)));
						printf("DT%f\n", float(mapDouble(rc->channels[rc->rcChannel.Throttle-1].pulseWidth, 1000, 2000, -1.0, 1.0)));
					}
				}
				else if (flightState->currentState == RollHold)
				{
					if (resetPidError)
					{
						resetPidError = false;
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
				}
				else if (flightState->currentState == AttitudeHold)
				{
					if (resetPidError)
					{
						resetPidError = false;
						pidRoll.resetError();
						pidPitch.resetError();
					}
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
				else if (flightState->currentState == AltHold) // alt holding using only throttle. if user push elevator, it wont work. this is just an example how to maintain alt via throttle
				{
					if (resetPidError)
					{
						pidAlt.resetError();
						pidThrottle.resetError();
						resetPidError = false;
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
				}
				checkForModeChange(rc, flightState);
				while((GetTick() > loop_timer) && ((GetTick() - loop_timer) % servoLoopTimeMs != 0));
				loop_timer = GetTick();
				last = loop_timer;
			}
			if ((GetTick() >= loop_timer) && ((GetTick() - loop_timer) % logLoopTimeMs == 0) && LOG == true)
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
				printf("log:hs%f\n", log.heading);
				printf("log:dw%f\n", log.distanceToWP);
				printf("log:bw%f\n", log.bearingToWP);
				while((GetTick() > loop_timer) && ((GetTick() - loop_timer) % logLoopTimeMs != 0));
				loop_timer = GetTick();
			}
			if ((GetTick() >= loop_timer) && ((GetTick() - loop_timer) % anglesRoutineTimems == 0))
			{
				if (SIMULATION_MODE)
				{
					/* TODO:
						when sending data over uart, it takes for the python code time to proccess it and sending back.
						while in simulation mode - the loops times will be bigger then the loops time using hardware
					*/
					// read 6dof simulation data and store
					requestData(huart1);
					huart1->Scanf(receivedBytes, SIMULATION_MSG_LEN); // must know x-plane return msg size
					Parser(receivedBytes, &UAVdata, &log);
					sendAck(huart1);
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
				while(GetTick() >= loop_timer) 
				{
					if (((GetTick() - loop_timer) % anglesRoutineTimems == 0) && ((GetTick() - last) % anglesRoutineTimems == 0))
						break;
				}
				last = GetTick();
				// because this is the fastast loop, dont "reset" loop_timer or else it will keep looping here
			}
		}
		else
		{
			continue;
		}
	}
	return 0;
}
