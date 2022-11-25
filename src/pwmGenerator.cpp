#include "pwmGenerator.h"

// globalChannel is used to grab data from parent class which hold the pins and other things
void pwmGenerator::enableTimerChannelPWM(uint8_t globalChannel)
{
	globalChannel -= 1; // user set the channel range 1 to N , but in the struct they are 0 to N-1
	uint8_t timerChannel = this->channels[globalChannel].timerChannel + 1; // +1 because when configuring the channels the user sends 1 to N, and in the configuration it is stored in array of 0 to N-1
	TIM_TypeDef *timer = this->channels[globalChannel].timer;
	uint8_t pinNum = 0;
	uint32_t gpioPinNum = this->channels[globalChannel].gpioPin;
	while (1)
	{
		gpioPinNum = gpioPinNum >> 1;
		pinNum += 1;
		if (gpioPinNum == 1)
			break;
	}
	if (this->channels[globalChannel].gpioPin <= 0x80)
	{
		//CRL
		uint32_t CNFBaseAddress = GPIO_CRL_CNF & 0x0C;
		uint32_t CNFOffset = CNFBaseAddress << (4U*pinNum);
		uint32_t cnfValue = CNFOffset & (0x08  << (4U*pinNum));
		
		this->channels[globalChannel].gpioPort->CRL &= ~(CNFOffset); // set CNF to 00
		
		uint32_t ModeBaseAddress = GPIO_CRL_MODE & 0x03;
		uint32_t ModeOffset = ModeBaseAddress << (4U*pinNum);
		uint32_t ModeValue = ModeOffset & (0x03  << (4U*pinNum));
		this->channels[globalChannel].gpioPort->CRL |= ModeValue | cnfValue; // set MODE TO 11 , CNF to 10
	}
	else
	{
		//CRH
		uint32_t CNFBaseAddress = GPIO_CRH_CNF & 0x0C;
		uint32_t CNFOffset = CNFBaseAddress << (4U*pinNum);
		uint32_t cnfValue = CNFOffset & (0x08  << (4U*pinNum));
		
		this->channels[globalChannel].gpioPort->CRH &= ~(CNFOffset); // set CNF to 00
		
		uint32_t ModeBaseAddress = GPIO_CRH_MODE & 0x03;
		uint32_t ModeOffset = ModeBaseAddress << (4U*pinNum);
		uint32_t ModeValue = ModeOffset & (0x03  << (4U*pinNum));
		this->channels[globalChannel].gpioPort->CRH |= ModeValue | cnfValue; // set MODE TO 11 , CNF to 10
	}
	if (timer == TIM4 || timer == TIM2 || timer == TIM3)
	{
		if (timer == TIM2) // needed while changing TIMER registers
			NVIC_DisableIRQ(TIM2_IRQn);
		else if (timer == TIM3) // needed while changing TIMER registers
			NVIC_DisableIRQ(TIM3_IRQn);
		else if (timer == TIM4) // needed while changing TIMER registers
			NVIC_DisableIRQ(TIM4_IRQn);
		timer->CR1 &= ~TIM_CR1_CEN;
		switch(timerChannel)
		{
			case 1:
				//configure pwm as active high and enable the signal 
				timer->CCER |= TIM_CCER_CC1E;
				timer->CR1 |= TIM_CR1_ARPE;//Auto reload preload enable ARPE
				//timer config for pwm mode down counting
				timer->CCMR1 &= (~TIM_CCMR1_CC1S_1 | ~TIM_CCMR1_CC1S_0);
				timer->CCMR1 &= ~TIM_CCMR1_OC1M;
				timer->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
				timer->DIER &= ~TIM_DIER_CC1IE;
				break;
			case 2:
				//configure pwm as active high and enable the signal 
				timer->CCER |= TIM_CCER_CC2E;
				timer->CR1 |= TIM_CR1_ARPE;//Auto reload preload enable ARPE
				//timer config for pwm mode down counting
				timer->CCMR1 &= (~TIM_CCMR1_CC2S_1 | ~TIM_CCMR1_CC2S_0);
				timer->CCMR1 &= ~TIM_CCMR1_OC1M;
				timer->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
				timer->DIER &= ~TIM_DIER_CC2IE;
				break;
			case 3:
				//configure pwm as active high and enable the signal 
				timer->CCER |= TIM_CCER_CC3E;
				timer->CR1 |= TIM_CR1_ARPE;//Auto reload preload enable ARPE
				//timer config for pwm mode down counting
				timer->CCMR2 &= (~TIM_CCMR2_CC3S_1 | ~TIM_CCMR2_CC3S_0);
				timer->CCMR2 &= ~TIM_CCMR2_OC3M;
				timer->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
				timer->DIER &= ~TIM_DIER_CC3IE;
				break;
			case 4:
				//configure pwm as active high and enable the signal 
				timer->CCER |= TIM_CCER_CC4E;
				timer->CR1 |= TIM_CR1_ARPE;//Auto reload preload enable ARPE
				//timer config for pwm mode down counting
				timer->CCMR2 &= (~TIM_CCMR2_CC4S_1 | ~TIM_CCMR2_CC4S_0);
				timer->CCMR2 &= ~TIM_CCMR2_OC4M;
				timer->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;
				timer->DIER &= ~TIM_DIER_CC4IE;
				break;
		}
			// set timer to 50Hz
			timer->PSC = 72 - 1;
			timer->ARR = 20000;
			
			//enable all registers by UG bit 
			timer->EGR |= TIM_EGR_UG;
			timer->CR1 |= TIM_CR1_CEN; //enable timer CEN = 1; 
	}
}

void pwmGenerator::sendPWM(uint8_t channel, int32_t pwmValue)
{
	channel -= 1;
	// instead of making many if statments - figure the corresponding ccr address and send the pwm value.
	uint32_t timerNBaseAddress = (uint32_t)this->channels[channel].timer;
	uint8_t timerChannelOffset = this->channels[channel].timerChannel*4;
	(*((uint32_t *)(timerNBaseAddress + CCR_BEGIN_ADDRESS + timerChannelOffset))) = pwmValue;
}

void pwmGenerator::Disco(RemoteController* rc)
{
	// loop through channels of rc (fixed wing - elevator,ailerons,rudder,throttle):
	// the code assumes that those channels created. if one of them isnt created (pwm or rc) the code will crash
	this->sendPWM(this->pwmChannel.Elevator, rc->getChannelPWM(rc->rcChannel.Elevator));
	this->sendPWM(this->pwmChannel.Aileron, rc->getChannelPWM(rc->rcChannel.Aileron));
//	this->sendPWM(this->pwmChannel.Rudder, rc->getChannelPWM(rc->rcChannel.Rudder));
//	this->sendPWM(this->pwmChannel.Throttle, rc->getChannelPWM(rc->rcChannel.Throttle));
}

int pwmGenerator::constrain(double pidResult)
{
	int pout_after = deg_to_ccr(pidResult);
	if (pout_after < SERVO_MIN_PWM)
			pout_after = SERVO_MIN_PWM;
	else if (pout_after > SERVO_MAX_PWM)
			pout_after = SERVO_MAX_PWM;
	return pout_after;
}
