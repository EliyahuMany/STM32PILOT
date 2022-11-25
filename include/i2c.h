#ifndef __I2C_H__
#define __I2C_H__

#include "stm32f10x.h"

class I2C_Handler
{
	public:
		I2C_TypeDef* registerStruct;
		bool remap;
		bool fastMode;
		uint32_t clockSpd;
		uint32_t sclFreq;
		I2C_Handler(I2C_TypeDef* registerStruct, bool remap, bool fastMode, uint32_t clockSpd, uint32_t sclFreq);
		void i2c_master_start();
		int i2c_master_transmitter(uint8_t slave_address);
		int i2c_master_send_byte(uint8_t data);
		int i2c_master_stop_sending();
		void i2c_master_stop_generation();
		int i2c_master_receiver(uint8_t slave_address);
		int i2c_master_read(uint8_t *data, uint8_t size);
		void i2c_read_reg(uint8_t slaveAddress, uint8_t regAddress, uint8_t *regData, uint8_t readSize);
		void i2c_write_reg(uint8_t slaveAddress, uint8_t regAddress, uint8_t sendData, uint8_t sendSize);
		void HangBusy();
	
	private:
		volatile uint8_t dataread;
};

#endif
