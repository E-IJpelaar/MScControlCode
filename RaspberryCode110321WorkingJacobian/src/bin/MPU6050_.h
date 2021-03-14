#ifndef MPU6050_H
#define MPU6050_H

//#include "Arduino.h"
//#include "Wire.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include "wiringPi/wiringPi.h"
#include "wiringPi/wiringPiI2C.h"

#define MPU6050_REG_START     0x3B
#define MPU6050_ADDR          0x68
#define MPU6050_SMPLRT_DIV    0x19
#define MPU6050_CONFIG        0x1A
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_REG_GYRO      0x43
#define MPU6050_WHO_AM_I      0x75
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_TEMP_H        0x41
#define MPU6050_TEMP_L        0x42

#define A_SCALE   16384
#define G_SCALE   131
#define T_SCALE   340
#define T_OFFSET  12412

#define PI 3.14159265358979323846

//#define accCoef 0.02
//#define gyroCoef 0.98

class MPU6050{
  public:
  
  int fd;
  float gyroXoffset;
  float gyroYoffset;
  float gyroZoffset;
  float rollBias;
  float pitchBias;
  float yawBias;

  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float temp;
  
  float angleAccX;
  float angleAccY;
  float angleGyroX;
  float angleGyroY;
  float angleGyroZ;
  
  float angleX;
  float angleY;
  float angleZ;
  
  float accCoef;
  float gyroCoef;
  
  float preInterval;
  
  bool begin();
  void update();
  void evalOffset();
  void evalBias();
  
  bool isConnected();

  float getTemp(){ return temp; };
  float getAccX(){ return accX; };
  float getAccY(){ return accY; };
  float getAccZ(){ return accZ; };
  float getGyroX(){ return gyroX; };
  float getGyroY(){ return gyroY; };
  float getGyroZ(){ return gyroZ; };
  float getRoll(){return angleX-rollBias; };
  float getPitch(){return angleY-pitchBias; };
  float getYaw(){return angleZ-yawBias; };
  
  void setAccCoef(float x){accCoef = x;};
  void setGyroCoef(float x){gyroCoef = x;};
  
};

#endif
