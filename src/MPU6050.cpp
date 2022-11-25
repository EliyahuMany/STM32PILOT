#include "MPU6050.h"
#include "math.h"

const double PI = 3.14159265358979323846;
const unsigned int calibrationLoop = 3000;
const float tau = 0.9996;

static float wrap(float angle,float limit){
  while (angle >  limit) angle -= 2*limit;
  while (angle < -limit) angle += 2*limit;
  return angle;
}

MPU6050::MPU6050(I2C_Handler *hi2c, uint8_t slaveAddress)
{
	this->slaveAddress = slaveAddress;
	this->hi2c = hi2c;
	this->accelXcalibrateFactor = 1;
	this->accelYcalibrateFactor = 1;
	this->accelZcalibrateFactor = 1;
	this->gyroXcalibrateFactor = 1;
	this->gyroYcalibrateFactor = 1;
	this->gyroZcalibrateFactor = 1;
	this->AnglesGyroscope_s.pitch = 0;
	this->AnglesGyroscope_s.roll = 0;
	this->AnglesGyroscope_s.yaw = 0;
}

void MPU6050::mpuInit()
{
	this->set_SMPLRT(7);
	for(int i=0;i<=10000;i++);
	this->set_CONFIG(0, 0);
	for(int i=0;i<=10000;i++);
	this->set_ACCEL_CONFIG(true, true, true, 2);
	for(int i=0;i<=10000;i++);
	this->set_GYRO_CONFIG(false, false, false, 1);
	for(int i=0;i<=10000;i++);
	this->set_INT_ENABLE(false, false, true);
	this->set_PWR_MGMT_1_On();
	this->calibrateAccel();
	this->calibrateGyro();
	this->AnglesGyroscope_s.pitch = this->AnglesAccelerometer_s.pitch;
	this->AnglesGyroscope_s.roll = this->AnglesAccelerometer_s.roll;
}

void MPU6050::set_SMPLRT(uint8_t sampleRate)
{
	hi2c->i2c_write_reg(slaveAddress, SMPLRT_DIV, sampleRate, 1);
}

void MPU6050::set_CONFIG(uint8_t fSync, uint8_t dlpfCfg)
{
	if (dlpfCfg > 6)
		dlpfCfg = 6;
	if (fSync > 7)
		fSync = 0;
	
	uint8_t regVal = 0x00;
	regVal = regVal | (fSync << 3) | dlpfCfg;
	hi2c->i2c_write_reg(slaveAddress, SMPLRT_DIV, regVal, 1);
}

void MPU6050::set_INT_ENABLE(bool enableFifo, bool enableMasterInt, bool enableDataRdyInt)
{
	uint8_t regVal = 0x00;
	if (enableFifo)
		regVal = regVal | 0x10;
	if (enableMasterInt)
		regVal = regVal | 0x08;
	if (enableDataRdyInt)
		regVal = regVal | 0x01;
	this->enableDataRdyInt = enableDataRdyInt;
	hi2c->i2c_write_reg(slaveAddress, INT_ENABLE, regVal, 1);
}

void MPU6050::set_PWR_MGMT_1_On()
{
	hi2c->i2c_write_reg(slaveAddress, PWR_MGMT_1, 0x08, 1);
}

void MPU6050::set_GYRO_CONFIG(bool xTest, bool yTest, bool zTest, uint8_t fsSel)
{
	uint8_t regVal = 0x00;
	if (xTest)
		regVal = regVal | 0x80;
	if (yTest)
		regVal = regVal | 0x40;
	if (zTest)
		regVal = regVal | 0x20;
	if (fsSel > 3)
		fsSel = 0;
	regVal = regVal | (fsSel << 3);
	switch (fsSel)
	{
		case (0):
			this->fsSel = FS_SEL_0;
			break;
		case (1):
			this->fsSel = FS_SEL_1;
			break;
		case (2):
			this->fsSel = FS_SEL_2;
			break;
		case (3):
			this->fsSel = FS_SEL_3;
			break;
	}
	hi2c->i2c_write_reg(slaveAddress, GYRO_CONFIG, regVal, 1);
	
}

void MPU6050::set_ACCEL_CONFIG(bool xTest, bool yTest, bool zTest, uint8_t afsSel)
{
	uint8_t regVal = 0x00;
	if (xTest)
		regVal = regVal | 0x80;
	if (yTest)
		regVal = regVal | 0x40;
	if (zTest)
		regVal = regVal | 0x20;
	if (afsSel > 3)
		afsSel = 0;
	regVal = regVal | (afsSel << 3);
	switch (afsSel)
	{
		case (0):
			this->afsSel = AFS_SEL_0;
			break;
		case (1):
			this->afsSel = AFS_SEL_1;
			break;
		case (2):
			this->afsSel = AFS_SEL_2;
			break;
		case (3):
			this->afsSel = AFS_SEL_3;
			break;
	}
	hi2c->i2c_write_reg(slaveAddress, ACCEL_CONFIG, regVal, 1);
	
}

void MPU6050::read_Accelerations()
{
	uint8_t accelerometrsValue[6];
	if (this->enableDataRdyInt)
	{
		uint8_t interruptStatus;
		while (1)
		{
		hi2c->i2c_read_reg(slaveAddress, INT_STATUS, &interruptStatus, 1);
		if ((interruptStatus & 0x01) == 0x01) // data is ready
			break;
		}
	}
	hi2c->i2c_read_reg(slaveAddress, ACCELEROMETERS, accelerometrsValue, 6); // read all accelerometes values
	this->MPU6050_s.accel_X = (int16_t) (accelerometrsValue[0] << 8 | accelerometrsValue[1]);
	this->MPU6050_s.accel_Y = (int16_t) (accelerometrsValue[2] << 8 | accelerometrsValue[3]);
	this->MPU6050_s.accel_Z = (int16_t) (accelerometrsValue[4] << 8 | accelerometrsValue[5]);
	this->MPU6050_s.ax = ((MPU6050_s.accel_X-this->accelXcalibrateFactor) / this->afsSel);
	this->MPU6050_s.ay = ((MPU6050_s.accel_Y-this->accelYcalibrateFactor) / this->afsSel);
	this->MPU6050_s.az = (MPU6050_s.accel_Z -this->accelZcalibrateFactor) / this->afsSel;
}

void MPU6050::read_Gyroscopes()
{
	uint8_t gyroscopesValue[6];
	if (this->enableDataRdyInt)
	{
		uint8_t interruptStatus;
		while (1)
		{
		hi2c->i2c_read_reg(slaveAddress, INT_STATUS, &interruptStatus, 1);
		if ((interruptStatus & 0x01) == 0x01)
			break;
		}
	}
	hi2c->i2c_read_reg(slaveAddress, GYROSCOPES, gyroscopesValue, 6); // read all gyroscopes values
	this->MPU6050_s.gyro_X = (int16_t) (gyroscopesValue[0] << 8 | gyroscopesValue[1]);
	this->MPU6050_s.gyro_Y = (int16_t) (gyroscopesValue[2] << 8 | gyroscopesValue[3]);
	this->MPU6050_s.gyro_Z = (int16_t) (gyroscopesValue[4] << 8 | gyroscopesValue[5]);
	this->MPU6050_s.gx = ((MPU6050_s.gyro_X-this->gyroXcalibrateFactor) / this->fsSel);
	this->MPU6050_s.gy = ((MPU6050_s.gyro_Y-this->gyroYcalibrateFactor) / this->fsSel);
	this->MPU6050_s.gz = ((MPU6050_s.gyro_Z-this->gyroZcalibrateFactor) / this->fsSel);
}

void MPU6050::calibrateAccel()
{
	double xAvg=0, yAvg=0, zAvg=0;
	for (int i=0;i <calibrationLoop;i++)
		this->read_Accelerations();
	for (int i=0; i<calibrationLoop; i++)
	{
		this->read_Accelerations();
		xAvg = xAvg + this->MPU6050_s.accel_X;
		yAvg = yAvg + this->MPU6050_s.accel_Y;
		zAvg = zAvg + this->MPU6050_s.accel_Z;
	}
	this->accelXcalibrateFactor = xAvg / calibrationLoop;
	this->accelYcalibrateFactor = yAvg / calibrationLoop;
	this->accelZcalibrateFactor = (zAvg / calibrationLoop) - this->afsSel;
}

void MPU6050::calibrateGyro()
{
	double xAvg=0, yAvg=0, zAvg=0;
	for (int i=0;i <calibrationLoop;i++)
		this->read_Gyroscopes();
	for (int i=0; i<calibrationLoop; i++)
	{
		this->read_Gyroscopes();
		xAvg = xAvg + this->MPU6050_s.gyro_X;
		yAvg = yAvg + this->MPU6050_s.gyro_Y;
		zAvg = zAvg + this->MPU6050_s.gyro_Z;
	}
	this->gyroXcalibrateFactor = xAvg / calibrationLoop;
	this->gyroYcalibrateFactor = yAvg / calibrationLoop;
	this->gyroZcalibrateFactor = zAvg / calibrationLoop;
}

/*
angles calculate may be more percise if using the raw data, and maybe faster too
*/
void MPU6050::anglesAccel()
{
	this->read_Accelerations();
	this->AnglesAccelerometer_s.roll = 180*atan2f(this->MPU6050_s.ax, sqrtf(((this->MPU6050_s.ay)*(this->MPU6050_s.ay)) + ((this->MPU6050_s.az)*(this->MPU6050_s.az))))/PI;
	this->AnglesAccelerometer_s.pitch = 180*atan2f(this->MPU6050_s.ay, sqrtf(((this->MPU6050_s.ax)*(this->MPU6050_s.ax)) + ((this->MPU6050_s.az)*(this->MPU6050_s.az))))/PI;
//	this->AnglesAccelerometer_s.pitch = atan2(this->MPU6050_s.ay, this->MPU6050_s.az) * (180 / PI);
//  this->AnglesAccelerometer_s.roll = atan2(this->MPU6050_s.ax, this->MPU6050_s.az) * (180 / PI);
}
void MPU6050::anglesGyro(uint16_t time) // rate in ms
{
	this->read_Gyroscopes();
	// to integrate the change in degrees, just need to sum the raw gyro value multiplied by the time passed from the sampling
	this->AnglesGyroscope_s.pitch = MPU6050_s.gx * (time/1000.0);
	this->AnglesGyroscope_s.roll = MPU6050_s.gy * (time/1000.0);
	this->AnglesGyroscope_s.yaw = MPU6050_s.gz * (time/1000.0);

}

void MPU6050::combinedAngles()
{
//	this->AnglesGyroscope_s.pitch = (this->AnglesGyroscope_s.pitch * 0.9996) + (this->AnglesAccelerometer_s.pitch * 0.0004);
//	this->AnglesGyroscope_s.roll = (this->AnglesGyroscope_s.roll * 0.9996) + (this->AnglesAccelerometer_s.roll * 0.0004);
	
//	this->CombinedAngles_s.pitch = (this->AnglesGyroscope_s.pitch * 0.9996) + (this->AnglesAccelerometer_s.pitch * 0.0004);
//	this->CombinedAngles_s.roll = (this->AnglesGyroscope_s.roll * 0.9996) + (this->AnglesAccelerometer_s.roll * 0.0004);
//	this->CombinedAngles_s.pitch = (this->CombinedAngles_s.pitch * 0.9) + (this->AnglesGyroscope_s.pitch * 0.1);
//	this->CombinedAngles_s.roll = (this->CombinedAngles_s.roll * 0.9) + (this->AnglesGyroscope_s.roll * 0.1);
	
	
	this->AnglesGyroscope_s.roll = (tau)*(this->AnglesGyroscope_s.roll) + (1-tau)*(this->AnglesAccelerometer_s.roll);
	this->CombinedAngles_s.roll = (0.9)*(this->CombinedAngles_s.roll - this->AnglesGyroscope_s.roll) + (0.1)*(this->AnglesAccelerometer_s.roll);
  
	this->AnglesGyroscope_s.pitch = (tau)*(this->AnglesGyroscope_s.pitch) + (1-tau)*(this->AnglesAccelerometer_s.pitch);
	this->CombinedAngles_s.pitch = (0.9)*(this->CombinedAngles_s.pitch - this->AnglesGyroscope_s.pitch) + (0.1)*(this->AnglesAccelerometer_s.pitch);
	
}

void MPU6050::compFilter()
{
	this->AnglesAccelerometer_s.pitch = 180*atan2f(this->MPU6050_s.ax, sqrtf(((this->MPU6050_s.ay)*(this->MPU6050_s.ay)) + ((this->MPU6050_s.az)*(this->MPU6050_s.az))))/PI;
	this->AnglesAccelerometer_s.roll = 180*atan2f(this->MPU6050_s.ay, sqrtf(((this->MPU6050_s.ax)*(this->MPU6050_s.ax)) + ((this->MPU6050_s.az)*(this->MPU6050_s.az))))/PI;
	this->CombinedAngles_s.yaw = this->AnglesGyroscope_s.yaw;
	// TODO: next two lines dampen the data, need to understand why to use it
	this->CombinedAngles_s.pitch = 0.9996*(this->CombinedAngles_s.pitch + this->AnglesGyroscope_s.pitch) + (this->AnglesAccelerometer_s.pitch * 0.0004);
	this->CombinedAngles_s.roll = 0.9996*(this->CombinedAngles_s.roll + this->AnglesGyroscope_s.roll) + (this->AnglesAccelerometer_s.roll * 0.0004);
}

/**
for simulation usage:
*/

double MPU6050::setCombinedRoll(double roll)
{
	this->CombinedAngles_s.roll = roll;
}

double MPU6050::setCombinedPitch(double pitch)
{
	this->CombinedAngles_s.pitch = pitch;
}

