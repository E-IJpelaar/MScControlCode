#include "MPU6050_.h"

////////////////////////////////////////////////////////////////////////
bool MPU6050::begin(){
  
  wiringPiSetup();
  
  fd = wiringPiI2CSetup(MPU6050_ADDR);
  
  if(fd < 0){
		return true;
	}
  
  wiringPiI2CWriteReg8(fd, MPU6050_SMPLRT_DIV,   0x00);
  wiringPiI2CWriteReg8(fd, MPU6050_CONFIG,       0x00);
  wiringPiI2CWriteReg8(fd, MPU6050_GYRO_CONFIG,  0x08);
  wiringPiI2CWriteReg8(fd, MPU6050_ACCEL_CONFIG, 0x00);
  wiringPiI2CWriteReg8(fd, MPU6050_PWR_MGMT_1,   0x01);
  
  angleGyroX = 0.0;
  angleGyroY = 0.0;
  rollBias   = 0.0;
  pitchBias  = 0.0;
  yawBias    = 0.0;
  accCoef    = 0.02;
  gyroCoef   = 0.98;
  //angleX = this->getAccAngleX();
  //angleY = this->getAccAngleY();
  preInterval = millis();
  
  //for(int i = 0; i < 300; i++){
  //  this->update();
  //}

  return false;
  //if (!isConnected()){
    //return false;
  //}
}

////////////////////////////////////////////////////////////////////////
void MPU6050::evalOffset(){
  
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;

  short rx, ry, rz;
  uint8_t msb, lsb;

  for(int i = 0; i < 3000; i++){

    msb = wiringPiI2CReadReg8(fd, MPU6050_REG_GYRO);
    lsb = wiringPiI2CReadReg8(fd, MPU6050_REG_GYRO+1);
    rx = msb << 8 | lsb;

    msb = wiringPiI2CReadReg8(fd, MPU6050_REG_GYRO+2);
    lsb = wiringPiI2CReadReg8(fd, MPU6050_REG_GYRO+3);
    ry = msb << 8 | lsb;

    msb = wiringPiI2CReadReg8(fd, MPU6050_REG_GYRO+4);
    lsb = wiringPiI2CReadReg8(fd, MPU6050_REG_GYRO+5);
    rz = msb << 8 | lsb;

    x += ((float)rx) / ((float)G_SCALE);
    y += ((float)ry) / ((float)G_SCALE);
    z += ((float)rz) / ((float)G_SCALE);

  }

  gyroXoffset = x / 3000;
  gyroYoffset = y / 3000;
  gyroZoffset = z / 3000;
}

////////////////////////////////////////////////////////////////////////
void MPU6050::evalBias(){
  
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
  
  for(int i = 0; i < 30; i++){
    
    update();
    x += getRoll();
    y += getPitch();
    z += getYaw();

  }
  
  rollBias  = x / 30;
  pitchBias = y / 30;
  yawBias   = z / 30;
}

////////////////////////////////////////////////////////////////////////
void MPU6050::update(){
  
  uint8_t msb = wiringPiI2CReadReg8(fd, MPU6050_REG_START);
  uint8_t lsb = wiringPiI2CReadReg8(fd, MPU6050_REG_START+1);
  short rawAccX = msb << 8 | lsb;

  msb = wiringPiI2CReadReg8(fd, MPU6050_REG_START+2);
  lsb = wiringPiI2CReadReg8(fd, MPU6050_REG_START+3);
  short rawAccY = msb << 8 | lsb;

  msb = wiringPiI2CReadReg8(fd, MPU6050_REG_START+4);
  lsb = wiringPiI2CReadReg8(fd, MPU6050_REG_START+5);
  short rawAccZ = msb << 8 | lsb;

  msb = wiringPiI2CReadReg8(fd, MPU6050_REG_START+6);
  lsb = wiringPiI2CReadReg8(fd, MPU6050_REG_START+7);
  short rawTemp = msb << 8 | lsb;

  msb = wiringPiI2CReadReg8(fd, MPU6050_REG_START+8);
  lsb = wiringPiI2CReadReg8(fd, MPU6050_REG_START+9);
  short rawGyroX = msb << 8 | lsb;

  msb = wiringPiI2CReadReg8(fd, MPU6050_REG_START+10);
  lsb = wiringPiI2CReadReg8(fd, MPU6050_REG_START+11);
  short rawGyroY = msb << 8 | lsb;

  msb = wiringPiI2CReadReg8(fd, MPU6050_REG_START+12);
  lsb = wiringPiI2CReadReg8(fd, MPU6050_REG_START+13);
  short rawGyroZ = msb << 8 | lsb;

  temp = ((float)(rawTemp + T_OFFSET))/ ((float) T_OFFSET);

  accX = ((float)rawAccX) / ((float)A_SCALE);
  accY = ((float)rawAccY) / ((float)A_SCALE);
  accZ = ((float)rawAccZ) / ((float)A_SCALE);

  angleAccX = atan2(accY, sqrt(accZ * accZ + accX * accX)) * 360 / 2.0 / PI;
  angleAccY = atan2(accX, sqrt(accZ * accZ + accY * accY)) * 360 / -2.0 / PI;

  gyroX = ((float)rawGyroX) / ((float)G_SCALE);
  gyroY = ((float)rawGyroY) / ((float)G_SCALE);
  gyroZ = ((float)rawGyroZ) / ((float)G_SCALE);

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

////////////////////////////////////////////////////////////////////////
bool MPU6050::isConnected(){
    return true; //read8(MPU6050_WHO_AM_I) == MPU6050_ADDR;
}
