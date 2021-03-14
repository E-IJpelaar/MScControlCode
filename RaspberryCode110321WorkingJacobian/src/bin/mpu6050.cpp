#include "wiringPi/wiringPi.h"
#include "wiringPi/wiringPiI2C.h"
#include "mpu6050.h"
#include <iostream>

#define PI 3.1415926

#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_INT_ENABLE   0x38
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_WHO_AM_I	 0x75

float accCoef = 0.02f;
float gyroCoef = 0.98f;

int fdmpu;
float preInterval = 0.0;
float gyroXoffset = 0.0;
float gyroYoffset = 0.0;
float gyroZoffset = 0.0;
float angleGyroX = 0.0;
float angleGyroY = 0.0;
float angleGyroZ = 0.0;
float angleX = 0.0;
float angleY = 0.0;
float angleZ = 0.0;

/**
 * Setup a PCA9685 device with wiringPi.
 *  
 * pinBase: 	Use a pinBase > 64, eg. 300
 * i2cAddress:	The default address is 0x40
 * freq:		Frequency will be capped to range [40..1000] Hertz. Try 50 for servos
 */
int mpu6050Setup(const int i2cAddress)
{
	
	// Check i2c address
	fdmpu = wiringPiI2CSetup(i2cAddress);
	if (fdmpu < 0){
		return fdmpu;
	}
	
	// Write to sample rate register
	wiringPiI2CWriteReg8(fdmpu, MPU6050_SMPLRT_DIV, 0x00); //was 0x07
	
	// Write to Configuration register
	wiringPiI2CWriteReg8(fdmpu, MPU6050_CONFIG, 0x00);
	
	// Write to Gyro Configuration register
	wiringPiI2CWriteReg8(fdmpu, MPU6050_GYRO_CONFIG, 0x08);
	
	// Write to Gyro Configuration register
	wiringPiI2CWriteReg8(fdmpu, MPU6050_ACCEL_CONFIG, 0x00);
	
	// Write to interrupt enable register 
	wiringPiI2CWriteReg8(fdmpu, MPU6050_INT_ENABLE, 0x01);
	
	// Write to power management register
	wiringPiI2CWriteReg8(fdmpu, MPU6050_PWR_MGMT_1, 0x01);	
	
	return fdmpu;
	
}

int readData(int address){
		
	//int high_byte,low_byte,value;
	//high_byte = wiringPiI2CReadReg8(fdmpu, address);
	//low_byte = wiringPiI2CReadReg8(fdmpu, address+1);
	//value = (high_byte << 8) | low_byte;
	//return value;
	int val;
	val = wiringPiI2CReadReg8(fdmpu, address);
	val = val << 8;
	val += wiringPiI2CReadReg8(fdmpu, address+1);
	if (val >= 0x8000)
		val = -(65536 - val);
		
	return val;
}

float mpu6050ReadAccX(){
		
	float RAW, A;
		
	RAW = readData(MPU6050_ACCEL_XOUT_H);
	A = RAW/16384.0;
	
	return A;
}

float mpu6050ReadAccY(){
		
	float RAW, A;
		
	RAW = readData(MPU6050_ACCEL_YOUT_H);
	A = RAW/16384.0;
	
	return A;
}

float mpu6050ReadAccZ(){
	
	float RAW, A;
		
	RAW = readData(MPU6050_ACCEL_ZOUT_H);
	A = RAW/16384.0;
	
	return A;
}

float mpu6050ReadGyroX(){
	
	float RAW, A;
		
	RAW = readData(MPU6050_GYRO_XOUT_H);
	//A = RAW/131.0;
	A = RAW/65.5;
	
	return A;
}

float mpu6050ReadGyroY(){
	
	float RAW, A;
		
	RAW = readData(MPU6050_GYRO_YOUT_H);
	//A = RAW/131.0;
	A = RAW/65.5;
	
	return A;
}

float mpu6050ReadGyroZ(){
	
	float RAW, A;
		
	RAW = readData(MPU6050_GYRO_ZOUT_H);
	//A = RAW/131.0;
	A = RAW/65.5;
	
	return A;
}

void mpu6050GyroOffset(){
	float x = 0, y = 0, z = 0;
	float rx, ry, rz;


	for(int i = 0; i < 3000; i++){

		rx = mpu6050ReadGyroX();
		ry = mpu6050ReadGyroY();
		rz = mpu6050ReadGyroZ();
	
		x += ((float)rx);
		y += ((float)ry);
		z += ((float)rz);
	}
  
	gyroXoffset = x / 3000;
	gyroYoffset = y / 3000;
	gyroZoffset = z / 3000;
	
	//cout << "done!" << endl;
}

int mpu6050IsConnected(){
	return wiringPiI2CReadReg8(fdmpu, MPU6050_WHO_AM_I);
}

void mpu6050Update(){

	float accX,accY,accZ;
	float gyroX,gyroY,gyroZ;
	float angleAccX, angleAccY;
	
	accX = mpu6050ReadAccX();
	accY = mpu6050ReadAccX();
	accZ = mpu6050ReadAccZ();
	gyroX = mpu6050ReadGyroX();
	gyroY = mpu6050ReadGyroY();
	gyroZ = mpu6050ReadGyroZ();
	
	angleAccX = atan2(accY, accZ + abs(accX)) * 360 / 2.0 / PI;
	angleAccY = atan2(accX, accZ + abs(accY)) * 360 / -2.0 / PI;

	gyroX -= gyroXoffset;
	gyroY -= gyroYoffset;
	gyroZ -= gyroZoffset;

	float interval = (millis() - preInterval) * 0.001;

	angleGyroX += gyroX * interval;
	angleGyroY += gyroY * interval;
	angleGyroZ += gyroZ * interval;

	angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
	angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
	angleZ = angleGyroZ;

	preInterval = millis();
}




