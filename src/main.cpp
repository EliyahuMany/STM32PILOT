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

//volatile uint32_t msTicks, _tick,loop_timer = 0 ;
volatile uint32_t loop_timer,last = 0;
volatile unsigned char led_on;
volatile uint32_t count, prev, tot, now = 0 ;
volatile uint32_t count2, prev2, tot2 = 0 ;
volatile bool press,int_flag = false;
const uint8_t anglesRoutineTimems = 4;
int loopCounter = 0;
int wtf = 0;
bool start = false;

double oldKp,oldKi,oldKd;


/**
RC Channels:

*/



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
	
	FlightState *flightState = new FlightState();
	
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
	
	PIDController pidRoll = PIDController(anglesRoutineTimems/1000.0, 1, 0.06, 0);
	PIDController pidPitch = PIDController(anglesRoutineTimems/1000.0, 1, 0.06, 0);
	

	TIM_TypeDef* timers[] = {TIM4};
	rc = new RemoteController(timers, sizeof(timers)/sizeof(timers[0]), 10);
	rc->RCInit();
	
	//TIM4->CCER |= TIM_CCER_CC1P;
	//TIM4->CCER |= TIM_CCER_CC1NE;
	

	flightState->currentState = Disco;
	start = false;
	while (true)
	{
		if (flightState->currentState == Disco)
		{
			//loop_timer = GetTick();
			// no need to check mpu and other things, just pass the user receiver values
			pwmHandler->Disco(rc);
			// check if switch changed position - change flight state
			// rc->
			if (rc->channels[rc->rcChannel.SwA-1].pulseWidth > 1800)
			{
				flightState->currentState = AttitudeHold;
				flightState->AttitudeHold(0, 0);
				start = false;
			}
			//rc->pp(); // print channels pwm
			while(GetTick() - loop_timer < 20);
			loop_timer = GetTick();
		}
		else if (flightState->currentState == AttitudeHold)
		{
			if (!start)
			{
				loop_timer = GetTick();
				start = true;
			}
			if ((GetTick() > loop_timer) && ((GetTick() - loop_timer) % 19 == 0))
			{
				
				
				if (rc->channels[rc->rcChannel.SwB-1].pulseWidth < 1100)
				{
					// roll attitude hold
					GPIOB->ODR ^= GPIO_ODR_ODR14;
					pidRoll.calculate(mpu.CombinedAngles_s.roll, flightState->GetAttitudeHoldParams().desiredRoll);
					pidRoll.constrain(-45, 45, -3, 3); // min and max values allowed on pidResult
					pwmHandler->sendPWM(rc->rcChannel.Aileron, pwmHandler->constrain(pidRoll.getPidResult()));
					pidPitch.calculate(mpu.CombinedAngles_s.pitch, 0);
					pidPitch.constrain(-45, 45, -3, 3); // min and max values allowed on pidResult
					pwmHandler->sendPWM(rc->rcChannel.Elevator, pwmHandler->constrain(pidPitch.getPidResult()));
					
					if (((990 < rc->channels[rc->rcChannel.Aileron-1].pulseWidth) && (rc->channels[rc->rcChannel.Aileron-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Aileron-1].pulseWidth) && (rc->channels[rc->rcChannel.Aileron-1].pulseWidth < 2010)))
					{
						// out of dead zone - set new pid setpoint
						float oldDesiredRoll = flightState->GetAttitudeHoldParams().desiredRoll;
						flightState->setAttitudeHoldRoll(oldDesiredRoll + map(rc->channels[rc->rcChannel.Aileron-1].pulseWidth, 1000, 2000, -45, 45) * 0.0075);
						if (flightState->GetAttitudeHoldParams().desiredRoll > 45)
							flightState->AttitudeHold(0, 45);
						else if (flightState->GetAttitudeHoldParams().desiredRoll < -45)
							flightState->AttitudeHold(0, -45);
					}
					
					// PID values change:
					if (rc->channels[rc->rcChannel.SwA-1].pulseWidth < 1400)
					{
						flightState->currentState = Disco;
					}
					else if ((rc->channels[rc->rcChannel.VarA-1].pulseWidth > 1100) && (rc->channels[rc->rcChannel.VarA-1].pulseWidth < 1500))
					{
						if (((990 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 2010)))
						{
							oldKp = pidRoll.getKp();
							//printf("%f\n",map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.0001);
							pidRoll.setKp(oldKp + map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.0001);
							//printf("P: %f\n",pidRoll.getKp());
						}
					}
					else if ((rc->channels[rc->rcChannel.VarA-1].pulseWidth > 1510) && (rc->channels[rc->rcChannel.VarA-1].pulseWidth < 1800))
					{
						if (((990 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 2010)))
						{
							oldKi = pidRoll.getKi();
							//printf("%f\n",map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.0001);
							pidRoll.setKi(oldKi + map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.0001);
							//printf("I: %f\n",pidRoll.getKi());
						}
					}
					else if ((rc->channels[rc->rcChannel.VarA-1].pulseWidth > 1810) && (rc->channels[rc->rcChannel.VarA-1].pulseWidth < 2010))
					{
						if (((990 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 2010)))
						{
							oldKd = pidRoll.getKd();
							//printf("%f\n",map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.0001);
							pidRoll.setKd(oldKd + map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.0001);
							//printf("D: %f\n",pidRoll.getKd());
						}
					}
				}
				else if ((rc->channels[rc->rcChannel.SwB-1].pulseWidth > 1400) && (rc->channels[rc->rcChannel.SwB-1].pulseWidth < 1800))
				{
					// pitch attitude hold
					GPIOB->ODR ^= GPIO_ODR_ODR14;
					pidPitch.calculate(mpu.CombinedAngles_s.pitch, flightState->GetAttitudeHoldParams().desiredPitch);
					pidPitch.constrain(-45, 45, -3, 3); // min and max values allowed on pidResult
					pidRoll.calculate(mpu.CombinedAngles_s.roll, 0);
					pidRoll.constrain(-45, 45, -3, 3); // min and max values allowed on pidResult
					pwmHandler->sendPWM(rc->rcChannel.Elevator, pwmHandler->constrain(pidPitch.getPidResult()));
					pwmHandler->sendPWM(rc->rcChannel.Aileron, pwmHandler->constrain(pidRoll.getPidResult()));
					
					if (((990 < rc->channels[rc->rcChannel.Elevator-1].pulseWidth) && (rc->channels[rc->rcChannel.Elevator-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Elevator-1].pulseWidth) && (rc->channels[rc->rcChannel.Elevator-1].pulseWidth < 2010)))
					{
						// out of dead zone - set new pid setpoint
						float oldDesiredPitch = flightState->GetAttitudeHoldParams().desiredPitch;
						flightState->setAttitudeHoldPitch(oldDesiredPitch + map(rc->channels[rc->rcChannel.Elevator-1].pulseWidth, 1000, 2000, -45, 45) * 0.0075);
						if (flightState->GetAttitudeHoldParams().desiredPitch > 45)
							flightState->AttitudeHold(0, 45);
						else if (flightState->GetAttitudeHoldParams().desiredPitch < -45)
							flightState->AttitudeHold(0, -45);
						printf("X: %f\n",flightState->GetAttitudeHoldParams().desiredPitch);
					}
					
					// PID values change:
					if (rc->channels[rc->rcChannel.SwA-1].pulseWidth < 1400)
					{
						flightState->currentState = Disco;
					}
					else if ((rc->channels[rc->rcChannel.VarA-1].pulseWidth > 1100) && (rc->channels[rc->rcChannel.VarA-1].pulseWidth < 1500))
					{
						if (((990 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 2010)))
						{
							oldKp = pidPitch.getKp();
							//printf("%f\n",map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.0001);
							pidPitch.setKp(oldKp + map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.001);
							printf("P: %f\n",pidPitch.getKp());
						}
					}
					else if ((rc->channels[rc->rcChannel.VarA-1].pulseWidth > 1510) && (rc->channels[rc->rcChannel.VarA-1].pulseWidth < 1800))
					{
						if (((990 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 2010)))
						{
							oldKi = pidPitch.getKi();
							//printf("%f\n",map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.0001);
							pidPitch.setKi(oldKi + map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.001);
							printf("I: %f\n",pidPitch.getKi());
						}
					}
					else if ((rc->channels[rc->rcChannel.VarA-1].pulseWidth > 1810) && (rc->channels[rc->rcChannel.VarA-1].pulseWidth < 2010))
					{
						if (((990 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Rudder-1].pulseWidth) && (rc->channels[rc->rcChannel.Rudder-1].pulseWidth < 2010)))
						{
							oldKd = pidPitch.getKd();
							//printf("%f\n",map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.0001);
							pidPitch.setKd(oldKd + map(rc->channels[rc->rcChannel.Rudder-1].pulseWidth, 1000, 2000, -20, 20) * 0.001);
							printf("D: %f\n",pidPitch.getKd());
						}
					}
				}
				else if ((rc->channels[rc->rcChannel.SwB-1].pulseWidth > 1800) && (rc->channels[rc->rcChannel.SwB-1].pulseWidth < 2010))
				{
					// both
					GPIOB->ODR ^= GPIO_ODR_ODR14;
					pidRoll.calculate(mpu.CombinedAngles_s.roll, flightState->GetAttitudeHoldParams().desiredRoll);
					pidRoll.constrain(-45, 45, -3, 3); // min and max values allowed on pidResult
					pwmHandler->sendPWM(rc->rcChannel.Aileron, pwmHandler->constrain(pidRoll.getPidResult()));
					
					if (((990 < rc->channels[rc->rcChannel.Aileron-1].pulseWidth) && (rc->channels[rc->rcChannel.Aileron-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Aileron-1].pulseWidth) && (rc->channels[rc->rcChannel.Aileron-1].pulseWidth < 2010)))
					{
						// out of dead zone - set new pid setpoint
						float oldDesiredRoll = flightState->GetAttitudeHoldParams().desiredRoll;
						flightState->setAttitudeHoldRoll(oldDesiredRoll + map(rc->channels[rc->rcChannel.Aileron-1].pulseWidth, 1000, 2000, -45, 45) * 0.0075);
						if (flightState->GetAttitudeHoldParams().desiredRoll > 45)
							flightState->AttitudeHold(0, 45);
						else if (flightState->GetAttitudeHoldParams().desiredRoll < -45)
							flightState->AttitudeHold(0, -45);
					}
					
					pidPitch.calculate(mpu.CombinedAngles_s.pitch, flightState->GetAttitudeHoldParams().desiredPitch);
					pidPitch.constrain(-45, 45, -3, 3); // min and max values allowed on pidResult
					pwmHandler->sendPWM(rc->rcChannel.Elevator, pwmHandler->constrain(pidPitch.getPidResult()));
					
					if (((990 < rc->channels[rc->rcChannel.Aileron-1].pulseWidth) && (rc->channels[rc->rcChannel.Aileron-1].pulseWidth < 1490)) || ((1510 < rc->channels[rc->rcChannel.Aileron-1].pulseWidth) && (rc->channels[rc->rcChannel.Aileron-1].pulseWidth < 2010)))
					{
						// out of dead zone - set new pid setpoint
						float oldDesiredPitch = flightState->GetAttitudeHoldParams().desiredPitch;
						flightState->setAttitudeHoldPitch(oldDesiredPitch + map(rc->channels[rc->rcChannel.Aileron-1].pulseWidth, 1000, 2000, -45, 45) * 0.0075);
						if (flightState->GetAttitudeHoldParams().desiredPitch > 45)
							flightState->AttitudeHold(0, 45);
						else if (flightState->GetAttitudeHoldParams().desiredPitch < -45)
							flightState->AttitudeHold(0, -45);
					}
				}
				
				
				
				
				//printf("%.1f\t%.1f\t%.1f\n", mpu.CombinedAngles_s.pitch, mpu.CombinedAngles_s.roll, mpu.CombinedAngles_s.yaw);
				
				while((GetTick() > loop_timer) && ((GetTick() - loop_timer) % 19 == 0));
				loop_timer = GetTick();
			}
			if (((GetTick() - loop_timer) % anglesRoutineTimems == 0))
			{
				GPIOB->ODR ^= GPIO_ODR_ODR13;
				mpu.anglesAccel();
				mpu.anglesGyro(anglesRoutineTimems);
				mpu.combinedAngles();
				while((GetTick() - loop_timer) % anglesRoutineTimems == 0);
			}
		}
	}
	return 0;
}
