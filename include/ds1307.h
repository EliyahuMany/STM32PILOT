#ifndef __DS1307_H__
#define __DS1307_H__

#include "stm32f10x.h"
#include "i2c.h"

#define SECONDS_REG 0x00
#define MINUTES_REG 0x01
#define HOURS_REG 0x02
#define DAY_REG 0x03
#define DATE_REG 0x04
#define MONTH_REG 0x05
#define YEAR_REG 0x06
#define CONTROL_REG 0x07

#define TWELVE_POS 0x40
#define TENTH_SECONDS_POS 0x70
#define TENTH_MINUTES_POS 0x70
#define TENTH_HOURS_AMPM_POS 0x10
#define TENTH_HOURS_POS 0x30
#define AMPM_POS 0x20
#define TENTH_DAY_POS 0x00
#define TENTH_DATE_POS 0x30
#define TENTH_MONTH_POS 0x10
#define TENTH_YEAR_POS 0xF0

class DS1307
{
	public:
		int time;
		I2C_Handler *hi2c;
		uint8_t slaveAddress;
	
		DS1307(I2C_Handler *hi2c, uint8_t slaveAddress);
		uint8_t readSeconds();
		uint8_t readMinutes();
		uint8_t readHours();
		bool readAMPM();
		uint8_t readDay();
		uint8_t readDate();
		uint8_t readMonth();
		uint8_t readYear();
		void setSeconds(uint8_t seconds);
		void setMinutes(uint8_t minutes);
		void setHours(uint8_t hour, bool twelve);
		void setDay(uint8_t day);
		void setDate(uint8_t date);
		void setMonth(uint8_t month);
		void setYear(uint8_t year);
	private:
		void dataWrite(uint8_t slaveAddress, uint8_t regAddress, uint8_t val);
		void dataRead(uint8_t slaveAddress, uint8_t regAddress, uint8_t *regVal, uint8_t size);
		uint8_t convert_BCD(uint8_t data, uint8_t tenthPos);
};

#endif
