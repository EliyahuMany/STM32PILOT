#ifndef __MPU6050_H__
#define __MPU6050_H__


#include "stm32f10x.h"
#include "i2c.h"


#define MPU6050_SLAVE_ADDRESS_AD0_0 0x68
#define MPU6050_SLAVE_ADDRESS_AD0_1 0x69
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define ACCELEROMETERS 0x3B
#define ACCELEROMETER_XOUT_HIGH 0x3B
#define ACCELEROMETER_XOUT_LOW 0x3C
#define ACCELEROMETER_YOUT_HIGH 0x3D
#define ACCELEROMETER_YOUT_LOW 0x3E
#define ACCELEROMETER_ZOUT_HIGH 0x3F
#define ACCELEROMETER_ZOUT_LOW 0x40
#define GYROSCOPES 0x43
#define GYROSCOPE_XOUT_HIGH 0x43
#define GYROSCOPE_XOUT_LOW 0x44
#define GYROSCOPE_YOUT_HIGH 0x45
#define GYROSCOPE_YOUT_LOW 0x46
#define GYROSCOPE_ZOUT_HIGH 0x47
#define GYROSCOPE_ZOUT_LOW 0x48
#define INT_ENABLE 0x38
#define INT_STATUS 0x3A
#define ACCEL_CONFIG 0x1C
#define AFS_SEL_0 (double)16384 // +- 2g scaling
#define AFS_SEL_1 (double)8192 // +- 4g scaling
#define AFS_SEL_2 (double)4096 // +- 8g scaling
#define AFS_SEL_3 (double)2048 // +- 16g scaling
#define GYRO_CONFIG 0x1B
#define FS_SEL_0 (double)131 // +- 250deg/S scaling
#define FS_SEL_1 (double)65.5 // +- 500deg/S scaling
#define FS_SEL_2 (double)32.8 // +- 1000deg/S scaling
#define FS_SEL_3 (double)16.4 // +- 2000deg/S scaling
#define PWR_MGMT_1 0X6B

class MPU6050
{
	public:
		I2C_Handler *hi2c;
		uint8_t slaveAddress;
		struct {
			int16_t accel_X;
			int16_t accel_Y;
			int16_t accel_Z;
			double ax;
			double ay;
			double az;
			
			int16_t gyro_X;
			int16_t gyro_Y;
			int16_t gyro_Z;
			double gx;
			double gy;
			double gz;
			
		} MPU6050_s;
		
		struct {
			float roll;
			float pitch;
			
		} AnglesAccelerometer_s;
		
		struct {
			float roll;
			float pitch;
			float yaw;
			
		} AnglesGyroscope_s;
		
		struct {
			float roll;
			float pitch;
			float yaw;
			
		} CombinedAngles_s;
		
		MPU6050(I2C_Handler *hi2c, uint8_t slaveAddress);
		void mpuInit();
		void set_SMPLRT(uint8_t sampleRate);
		void set_CONFIG(uint8_t fSync, uint8_t dlpfCfg);
		void read_Accelerations();
		void read_Gyroscopes();
		void set_INT_ENABLE(bool enableFifo, bool enableMasterInt, bool enableDataRdyInt);
		void set_ACCEL_CONFIG(bool xTest, bool yTest, bool zTest, uint8_t afsSel);
		void set_GYRO_CONFIG(bool xTest, bool yTest, bool zTest, uint8_t fsSel);
		void calibrateAccel();
		void calibrateGyro();
		void anglesAccel();
		void anglesGyro(uint16_t time);
		void combinedAngles();
		void compFilter();
		void set_PWR_MGMT_1_On();
		/** Simulation usage*/
		double setCombinedRoll(double roll);
		double setCombinedPitch(double pitch);
	private:
		bool enableDataRdyInt;
		double afsSel; // used in ACCCEL_CONFIG register settings - scaling factor of acceleration
		double fsSel; // used in GYRO_CONFIG register settings - scaling factor of gyro
		double accelXcalibrateFactor;
		double accelYcalibrateFactor;
		double accelZcalibrateFactor;
		double gyroXcalibrateFactor;
		double gyroYcalibrateFactor;
		double gyroZcalibrateFactor;
};

#endif
