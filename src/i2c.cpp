#include "i2c.h"
#include "SysTickHandler.h"

uint16_t errorCount = 0;

I2C_Handler::I2C_Handler(I2C_TypeDef* registerStruct, bool remap, bool fastMode, uint32_t clockSpd, uint32_t sclFreq)
	{
		// TODO: check if I2C peripherial already initiated, if so - return the one which is alreay exist.
		this->registerStruct = registerStruct;
		this->remap = remap;
		this->fastMode = fastMode;
		this->clockSpd = clockSpd;
		this->sclFreq = sclFreq;
		
		// Enable port B clock
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
		// Enable Alternate-Function Clock
		RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
		
		if (this->registerStruct == I2C1)
		{
			if (this->remap)
			{
				AFIO->MAPR |= AFIO_MAPR_I2C1_REMAP;
				// Using PB8=SCL, PB9=SDA MODE and CNF
				if ((this->clockSpd/1000000 > 2) && (this->clockSpd/1000000 <= 10))
				{
					// need to use mode 01 (max 10MHZ)
					GPIOB->CRH &= (~GPIO_CRH_MODE8_0 & ~GPIO_CRH_MODE9_0);
					GPIOB->CRH |= GPIO_CRH_MODE8_1 | GPIO_CRH_MODE9_1;
				}
				else if (this->clockSpd/1000000 <= 2)
				{
					// need to use mode 10 (max 2MHz)
					GPIOB->CRH |= GPIO_CRH_MODE8_0 | GPIO_CRH_MODE9_0;
					GPIOB->CRH &= (~GPIO_CRH_MODE8_1 & ~GPIO_CRH_MODE9_1);
				}
				else
				{
					// need to use mode 11 (max 50MHz)
					GPIOB->CRH |= GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1;
					GPIOB->CRH |= GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1;
				}
				// set both CNF as 11 (Alternate-Function Open-drain)
				GPIOB->CRH |= GPIO_CRH_CNF8_0 | GPIO_CRH_CNF8_1;
				GPIOB->CRH |= GPIO_CRH_CNF9_0 | GPIO_CRH_CNF9_1;
			}
			else
			{
				AFIO->MAPR &= ~AFIO_MAPR_I2C1_REMAP;
				// Using PB6=SCL, PB7=SDA MODE and CNF
				if ((this->clockSpd/1000000 > 2) && (this->clockSpd/1000000 <= 10))
				{
					// need to use mode 01 (max 10MHZ)
					GPIOB->CRL &= (~GPIO_CRL_MODE6_0 & ~GPIO_CRL_MODE7_0);
					GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE7_1;
				}
				else if (this->clockSpd/1000000 <= 2)
				{
					// need to use mode 10 (max 2MHz)
					GPIOB->CRL |= GPIO_CRL_MODE6_0 | GPIO_CRL_MODE7_0;
					GPIOB->CRL &= (~GPIO_CRL_MODE6_1 & ~GPIO_CRL_MODE7_1);
				}
				else
				{
					// need to use mode 11 (max 50MHz)
					GPIOB->CRL |= GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1;
					GPIOB->CRL |= GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1;
				}
				// set both CNF as 11 (Alternate-Function Open-drain)
				GPIOB->CRL |= GPIO_CRL_CNF6_0 | GPIO_CRL_CNF6_1;
				GPIOB->CRL |= GPIO_CRL_CNF7_0 | GPIO_CRL_CNF7_1;
			}
			// Enable I2C1
			RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
		}
		else
		{
			// Configuration of I2C2
			// Using PB10=SCL, PB11=SDA MODE and CNF
				if ((this->clockSpd/1000000 > 2) && (this->clockSpd/1000000 <= 10))
				{
					// need to use mode 01 (max 10MHZ)
					GPIOB->CRH &= (~GPIO_CRH_MODE10_0 & ~GPIO_CRH_MODE11_0);
					GPIOB->CRH |= GPIO_CRH_MODE10_1 | GPIO_CRH_MODE11_1;
				}
				else if (this->clockSpd/1000000 <= 2)
				{
					// need to use mode 10 (max 2MHz)
					GPIOB->CRH |= GPIO_CRH_MODE10_0 | GPIO_CRH_MODE11_0;
					GPIOB->CRH &= (~GPIO_CRH_MODE10_1 & ~GPIO_CRH_MODE11_1);
				}
				else
				{
					// need to use mode 11 (max 50MHz)
					GPIOB->CRH |= GPIO_CRH_MODE10_0 | GPIO_CRH_MODE10_1;
					GPIOB->CRH |= GPIO_CRH_MODE11_0 | GPIO_CRH_MODE11_1;
				}
				// set both CNF as 11 (Alternate-Function Open-drain)
				GPIOB->CRH |= GPIO_CRH_CNF10_0 | GPIO_CRH_CNF10_1;
				GPIOB->CRH |= GPIO_CRH_CNF11_0 | GPIO_CRH_CNF11_1;
				
				RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
		}
		
		// configure I2C clock
		if (this->clockSpd/1000000 > 32)
		{
			// APB1 max Freq is 24/32 MHz
			// TODO: add ability to check what is the APB1 Freq, and by that change the user clock freq
			this->clockSpd = 32000000;
		}
		else if (this->clockSpd/1000000 < 2)
		{
			this->clockSpd = 2000000;
		}
		this->registerStruct->CR1 &= ~I2C_CR1_PE;
		this->registerStruct->CR1 |= I2C_CR1_SWRST;
		for (int i=0; i<1000; i++);
		this->registerStruct->CR1 &= ~I2C_CR1_SWRST;
		this->registerStruct->CR2 |= (this->clockSpd/1000000);
		if (this->fastMode)
		{
			this->registerStruct->CCR |= I2C_CCR_FS;
			// TODO: LEARN HOW FAST MODE WORK AND IMPLEMANT
		}
		else
		{
			this->registerStruct->CCR &= ~I2C_CCR_FS;
			this->registerStruct->CCR |= (I2C_CCR_CCR & (this->clockSpd/this->sclFreq));
			this->registerStruct->TRISE &= 0x0;
			this->registerStruct->TRISE |= (I2C_TRISE_TRISE & (this->clockSpd/1000000 + 1));
		}
		
		this->registerStruct->CR1 |= I2C_CR1_PE;
		if((this->registerStruct->SR2 & I2C_SR2_BUSY) == I2C_SR2_BUSY)
			i2c_master_stop_generation();
	}

/*
Setting the START bit causes the interface to generate a Start condition and to switch to 
Master mode (MSL bit set) when the BUSY bit is cleared.
*/
void I2C_Handler::i2c_master_start()
{
	uint32_t Tickstart = GetTick();
//	if((this->registerStruct->SR2 & I2C_SR2_BUSY) == I2C_SR2_BUSY)
//		i2c_master_stop_generation();
	this->registerStruct->CR1 |= I2C_CR1_START;
	while(!(this->registerStruct->SR1 & I2C_SR1_SB))
	{
		// Check if certain time passed. if so - return.
		if ((GetTick() - Tickstart) > 100)
		{
			errorCount += 1;
			break;
		}
	}
	dataread = this->registerStruct->SR1;
}

int I2C_Handler::i2c_master_transmitter(uint8_t slave_address)
{
	uint32_t Tickstart = GetTick();
	this->registerStruct->DR = slave_address << 1; // reset LSB
	while(!(this->registerStruct->SR1 &= I2C_SR1_ADDR))
	{
		
		if ((this->registerStruct->SR1 &= I2C_SR1_AF) == I2C_SR1_AF)
		{
			return 1;
		}
		if ((GetTick() - Tickstart) > 2)
		{
			errorCount += 1;
			return 1;
		}
		
	}
	dataread = this->registerStruct->SR1;
	dataread = this->registerStruct->SR2;
	return 0;
}

int I2C_Handler::i2c_master_send_byte(uint8_t data)
{
	uint32_t Tickstart = GetTick();
	this->registerStruct->DR = data;
	while(!(this->registerStruct->SR1 & I2C_SR1_TXE)) // waits until the transmission is ended
	{
		if ((this->registerStruct->SR1 &= I2C_SR1_AF) == I2C_SR1_AF)
		{
			return 1;
		}
		if ((GetTick() - Tickstart) > 2)
		{
			errorCount += 1;
			return 1;
		}
	}
	return 0;
}
int I2C_Handler::i2c_master_stop_sending()
{
	uint32_t Tickstart = GetTick();
	while(!(this->registerStruct->SR1 & I2C_SR1_BTF)) // waits until the transmission is ended
	{
		if ((this->registerStruct->SR1 &= I2C_SR1_AF) == I2C_SR1_AF)
		{
			return 1;
		}
		if ((GetTick() - Tickstart) > 2)
		{
			errorCount += 1;
			return 1;
		}
	}
	return 0;
}

void I2C_Handler::i2c_master_stop_generation()
{
	this->registerStruct->CR1 |= I2C_CR1_STOP;
}

int I2C_Handler::i2c_master_receiver(uint8_t slave_address)
{
	uint32_t Tickstart = GetTick();
	slave_address = slave_address << 1; // set LSB
	this->registerStruct->DR = slave_address + 1; // set LSB
	
	while (!READ_BIT(this->registerStruct->SR1, I2C_SR1_ADDR))             // wait until address has been sent
	{
			if (READ_BIT(this->registerStruct->SR1, I2C_SR1_AF))
			{
					// did not receive ACK after address
					return 1;
			}
			if ((GetTick() - Tickstart) > 2)
			{
				errorCount += 1;
				return 1;
			}
	}

	dataread = this->registerStruct->SR1;
	dataread = this->registerStruct->SR2;
	return 0;

}

int I2C_Handler::i2c_master_read(uint8_t *data, uint8_t size)
{
	this->registerStruct->CR1 &= ~I2C_CR1_POS;
	this->registerStruct->CR1 |= I2C_CR1_ACK; // enable getting ACK after each byte received
	if (size == 1)
	{
		// EV6_3 (after clearin ADDR)
		this->registerStruct->CR1 &= ~I2C_CR1_ACK;
		i2c_master_stop_generation();
		uint32_t Tickstart = GetTick();
		while(!(this->registerStruct->SR1 & I2C_SR1_RXNE))
		{
//			if ((GetTick() - Tickstart) > 10)
//			{
//				errorCount += 1;
//				return 1;
//			}
		}
		*data= this->registerStruct->DR;
	}
	else if (size == 2)
	{
		// EV6_1 (after clearin ADDR)
		this->registerStruct->CR1 |= I2C_CR1_POS;
		
		this->registerStruct->CR1 &= ~I2C_CR1_ACK;
		uint32_t Tickstart = GetTick();
		while(!(this->registerStruct->SR1 & I2C_SR1_BTF))
		{
//			if ((GetTick() - Tickstart) > 10)
//			{
//				errorCount += 1;
//				return 1;
//			}
		}
		i2c_master_stop_generation();
		*data = this->registerStruct->DR;
		// TODO: check if next address is allocated to prevent exceptions/data overwriting
		data = data + 1; // move the pointer to the next mem address
		*data = this->registerStruct->DR;
		
	}
	else
	{
		// TODO: check if next address is allocated to prevent exceptions/data overwriting
		while (size > 3)
		{
			while(!(this->registerStruct->SR1 & I2C_SR1_RXNE));
			*data = this->registerStruct->DR;
			data = data + 1; // move the pointer to the next mem address
			size--;
		}
		while(!(this->registerStruct->SR1 & I2C_SR1_RXNE)); // wait for dataN-1
		while(!(this->registerStruct->SR1 & I2C_SR1_BTF)); // dataN-2 in DR register and dataN-1 in the shift register
		this->registerStruct->CR1 &= ~I2C_CR1_ACK;
		*data = this->registerStruct->DR; // read dataN-2
		data = data + 1; // move the pointer to the next mem address
		size--;
		i2c_master_stop_generation();
		*data = this->registerStruct->DR; // read dataN-1
		data = data + 1; // move the pointer to the next mem address
		size--;
		while(!(this->registerStruct->SR1 & I2C_SR1_RXNE));
		*data = this->registerStruct->DR; // read dataN
	}
	return 0;
}

/**
read_reg gets the reg address to read from to regAddress + readSize
*/
void I2C_Handler::i2c_read_reg(uint8_t slaveAddress, uint8_t regAddress, uint8_t *regData, uint8_t readSize)
{
	this->i2c_master_start();
	this->i2c_master_transmitter(slaveAddress);
	this->i2c_master_send_byte(regAddress);
	this->i2c_master_start();
	this->i2c_master_receiver(slaveAddress);
	this->i2c_master_read(regData, readSize);
	this->i2c_master_stop_generation();
	for(int i=0;i<10;i++); // added this to reduce errors, when not used many of the while loops inside of each func above get stuck.
	// TODO: find a way to solve this - maybe need to check if stop completed
}

void I2C_Handler::i2c_write_reg(uint8_t slaveAddress, uint8_t regAddress, uint8_t sendData, uint8_t sendSize)
{
	this->i2c_master_start();
	this->i2c_master_transmitter(slaveAddress);
	this->i2c_master_send_byte(regAddress);
	while (sendSize>0)
	{
		this->i2c_master_send_byte(sendData);
		sendSize--;
	}
	this->i2c_master_stop_sending();
	this->i2c_master_stop_generation();
}

void I2C_Handler::HangBusy()
{
	this->registerStruct->CR1 &= ~I2C_CR1_PE;
	if (this->remap)
	{
		// Using PB8=SCL, PB9=SDA MODE and CNF
		if ((this->clockSpd/1000000 > 2) && (this->clockSpd/1000000 <= 10))
		{
			// need to use mode 01 (max 10MHZ)
			GPIOB->CRL &= (~GPIO_CRH_MODE8_0 & ~GPIO_CRH_MODE9_0);
			GPIOB->CRL |= GPIO_CRH_MODE8_1 | GPIO_CRH_MODE9_1;
		}
		else if (this->clockSpd/1000000 <= 2)
		{
			// need to use mode 10 (max 2MHz)
			GPIOB->CRL |= GPIO_CRH_MODE8_0 | GPIO_CRH_MODE9_0;
			GPIOB->CRL &= (~GPIO_CRH_MODE8_1 & ~GPIO_CRH_MODE9_1);
		}
		else
		{
			// need to use mode 11 (max 50MHz)
			GPIOB->CRL |= GPIO_CRH_MODE8_0 | GPIO_CRH_MODE8_1;
			GPIOB->CRL |= GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1;
		}
		// set both CNF as 11 (Alternate-Function Open-drain)
		GPIOB->CRL |= GPIO_CRH_CNF8_0;
		GPIOB->CRL &= ~GPIO_CRH_CNF8_1;
		
		GPIOB->CRL |= GPIO_CRH_CNF9_0;
		GPIOB->CRL &= ~GPIO_CRH_CNF9_1;
		
		GPIOB->ODR |= GPIO_ODR_ODR8 | GPIO_ODR_ODR9;
	}
	else
	{
		// Using PB6=SCL, PB7=SDA MODE and CNF
		if ((this->clockSpd/1000000 > 2) && (this->clockSpd/1000000 <= 10))
		{
			// need to use mode 01 (max 10MHZ)
			GPIOB->CRL &= (~GPIO_CRL_MODE6_0 & ~GPIO_CRL_MODE7_0);
			GPIOB->CRL |= GPIO_CRL_MODE6_1 | GPIO_CRL_MODE7_1;
		}
		else if (this->clockSpd/1000000 <= 2)
		{
			// need to use mode 10 (max 2MHz)
			GPIOB->CRL |= GPIO_CRL_MODE6_0 | GPIO_CRL_MODE7_0;
			GPIOB->CRL &= (~GPIO_CRL_MODE6_1 & ~GPIO_CRL_MODE7_1);
		}
		else
		{
			// need to use mode 11 (max 50MHz)
			GPIOB->CRL |= GPIO_CRL_MODE6_0 | GPIO_CRL_MODE6_1;
			GPIOB->CRL |= GPIO_CRL_MODE7_0 | GPIO_CRL_MODE7_1;
		}
		// set both CNF as 11 (Alternate-Function Open-drain)
		GPIOB->CRL |= GPIO_CRL_CNF6_0;
		GPIOB->CRL &= ~GPIO_CRL_CNF6_1;
		
		GPIOB->CRL |= GPIO_CRL_CNF7_0;
		GPIOB->CRL &= ~GPIO_CRL_CNF7_1;
		
		GPIOB->ODR |= GPIO_ODR_ODR6 | GPIO_ODR_ODR7;
		
		while(!(GPIOB->IDR & (GPIO_IDR_IDR6 | GPIO_IDR_IDR7)));
		
		// only sda to low level
		GPIOB->CRL |= GPIO_CRL_CNF7_0;
		GPIOB->CRL &= ~GPIO_CRL_CNF7_1;
		GPIOB->ODR &= ~GPIO_ODR_ODR7;
		while((GPIOB->IDR & GPIO_IDR_IDR7));
		
		// scl low level
		GPIOB->CRL |= GPIO_CRL_CNF6_0;
		GPIOB->CRL &= ~GPIO_CRL_CNF6_1;
		GPIOB->ODR &= ~GPIO_ODR_ODR6;
		while((GPIOB->IDR & GPIO_IDR_IDR6));
		
		// scl high level
		GPIOB->CRL |= GPIO_CRL_CNF6_0;
		GPIOB->CRL &= ~GPIO_CRL_CNF6_1;
		GPIOB->ODR |= GPIO_ODR_ODR6;
		while(!(GPIOB->IDR & GPIO_IDR_IDR6));
		
		// sda high level
		GPIOB->CRL |= GPIO_CRL_CNF7_0;
		GPIOB->CRL &= ~GPIO_CRL_CNF7_1;
		GPIOB->ODR |= GPIO_ODR_ODR7;
		while(!(GPIOB->IDR & GPIO_IDR_IDR7));
		
		GPIOB->CRL |= GPIO_CRL_CNF6_0 | GPIO_CRL_CNF6_1;
		GPIOB->CRL |= GPIO_CRL_CNF7_0 | GPIO_CRL_CNF7_1;
		
		this->registerStruct->CR1 |= I2C_CR1_SWRST;
		for (int i=0; i<1000; i++);
		this->registerStruct->CR1 &= ~I2C_CR1_SWRST;
		this->registerStruct->CR1 |= I2C_CR1_PE;
		
		
	}
	
}
