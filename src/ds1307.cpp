#include "ds1307.h"
#include "i2c.h"


DS1307::DS1307(I2C_Handler *hi2c, uint8_t slaveAddress)
{
	this->slaveAddress = slaveAddress;
	this->hi2c = hi2c;
}

uint8_t DS1307::readSeconds()
{
	uint8_t seconds;
	hi2c->i2c_read_reg(slaveAddress, SECONDS_REG, &seconds, 1);
	return convert_BCD(seconds, TENTH_SECONDS_POS);
}

uint8_t DS1307::readMinutes()
{
	uint8_t minutes;
	hi2c->i2c_read_reg(slaveAddress, MINUTES_REG, &minutes, 1);
	return convert_BCD(minutes, TENTH_MINUTES_POS);
}
/**
	return the hour value.
	if the user choosed to use am/pm mode - it will return value from 1-12
	it is the user responsebility to get the am/pm value from readAMPM() function
*/
uint8_t DS1307::readHours()
{
	uint8_t hour;
	hi2c->i2c_read_reg(slaveAddress, HOURS_REG, &hour, 1);
	if (hour & AMPM_POS)
	{
		return convert_BCD(hour, TENTH_HOURS_AMPM_POS);
	}
	return convert_BCD(hour, TENTH_HOURS_POS);
}
/**
	true = PM
	false = AM
*/
bool DS1307::readAMPM()
{
	uint8_t hour;
	hi2c->i2c_read_reg(slaveAddress, HOURS_REG, &hour, 1);
	if (hour & AMPM_POS)
	{
		return true;
	}
	return false;
}

uint8_t DS1307::readDay()
{
	uint8_t day;
	hi2c->i2c_read_reg(slaveAddress, DAY_REG, &day, 1);
	return convert_BCD(day, TENTH_DAY_POS);
}

uint8_t DS1307::readDate()
{
	uint8_t date;
	hi2c->i2c_read_reg(slaveAddress, DATE_REG, &date, 1);
	return convert_BCD(date, TENTH_DATE_POS);
}

uint8_t DS1307::readMonth()
{
	uint8_t month;
	hi2c->i2c_read_reg(slaveAddress, MONTH_REG, &month, 1);
	return convert_BCD(month, TENTH_MONTH_POS);
}

uint8_t DS1307::readYear()
{
	uint8_t year;
	hi2c->i2c_read_reg(slaveAddress, YEAR_REG, &year, 1);
	return convert_BCD(year, TENTH_YEAR_POS);
}


/**
	setters:
*/
void DS1307::setSeconds(uint8_t seconds)
{
	if (seconds > 59)
		seconds = 0;
	uint8_t seconds_BCD = (seconds/10 << 4) | seconds%10;
	hi2c->i2c_write_reg(slaveAddress, SECONDS_REG, seconds_BCD, 1);
}

void DS1307::setMinutes(uint8_t minutes)
{
	if (minutes > 59)
		minutes = 0;
	uint8_t minutes_BCD = (minutes/10 << 4) | minutes%10;
	hi2c->i2c_write_reg(slaveAddress, MINUTES_REG, minutes_BCD, 1);
}

void DS1307::setHours(uint8_t hour, bool twelve)
{
	if (hour >= 24)
		hour = 0;
		
	uint8_t hour_BCD = twelve ? TWELVE_POS:0x00 | ((hour/10 << 4) | hour%10);
	hi2c->i2c_write_reg(slaveAddress, HOURS_REG, hour_BCD, 1);
}

void DS1307::setDay(uint8_t day)
{
	if ((day < 1) && (day > 7))
		day = 1;
	hi2c->i2c_write_reg(slaveAddress, DAY_REG, day, 1);
}

void DS1307::setDate(uint8_t date)
{
	if (date < 1 && date > 31)
		date = 1;
	uint8_t date_BCD = ((date/10 << 4) | date%10);
	hi2c->i2c_write_reg(slaveAddress, DATE_REG, date_BCD, 1);
}

void DS1307::setMonth(uint8_t month)
{
	if (month < 1 && month > 12)
		month = 1;
	uint8_t month_BCD = ((month/10 << 4) | month%10);
	hi2c->i2c_write_reg(slaveAddress, MONTH_REG, month_BCD, 1);
}

void DS1307::setYear(uint8_t year)
{
	if (year > 99)
		year = 0;
	uint8_t year_BCD = ((year/10 << 4) | year%10);
	hi2c->i2c_write_reg(slaveAddress, YEAR_REG, year_BCD, 1);
}

uint8_t DS1307::convert_BCD(uint8_t data, uint8_t tenthPos)
{
	/*
	Decimal digit	BCD
						8	4	2	1
			0			0	0	0	0
			1			0	0	0	1
			2			0	0	1	0
			3			0	0	1	1
			4			0	1	0	0
			5			0	1	0	1
			6			0	1	1	0
			7			0	1	1	1
			8			1	0	0	0
			9			1	0	0	1
	*/
	uint8_t rest = data & 0x0F;
	uint8_t tenmul = (data >> 4) & (tenthPos >> 4); // in order to ignore some bits, need to multiply only on the positions that we care about
	uint8_t total = (10 * tenmul) + rest;
	return total;
}
