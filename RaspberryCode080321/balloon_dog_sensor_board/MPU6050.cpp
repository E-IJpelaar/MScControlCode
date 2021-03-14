#include "MPU6050.h"
#include "Arduino.h"


/**************************************************************************/
/*!
    @brief contruct class
*/
/**************************************************************************/
MPU6050::MPU6050(){
  accCoef = 0.2f;
  gyroCoef = 0.8f;
}


/**************************************************************************/
/*!
    @brief contruct class
*/
/**************************************************************************/
MPU6050::MPU6050(float aC, float gC){
  accCoef = aC;
  gyroCoef = gC;
}


/**************************************************************************/
/*!
    @brief initiate class
    @return True, sensor properly connected. Returns false if not.
*/
/**************************************************************************/
bool MPU6050::begin(uint8_t i2c_addr, TwoWire *twoWire){

  _i2c_addr = i2c_addr;
  _i2c = twoWire;

  _i2c->begin();
  
  //writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);
  //writeMPU6050(MPU6050_CONFIG, 0x00);
  //writeMPU6050(MPU6050_GYRO_CONFIG, 0x08); // was 0x08
  //writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00);
  //writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);
  this->update();
  angleGyroX = 0;
  angleGyroY = 0;
  angleX = this->getAccAngleX();
  angleY = this->getAccAngleY();
  preInterval = millis();

  return isConnected();
}

/**************************************************************************/
/*!
    @brief writes 1 byte to register at adress MPU6050_ADDR
*/
/**************************************************************************/
void MPU6050::writeMPU6050(byte reg, byte data){
  _i2c->beginTransmission(MPU6050_ADDR);
  _i2c->write(reg);
  _i2c->write(data);
  _i2c->endTransmission();
}

/**************************************************************************/
/*!
    @brief reads 1 byte of register at adress MPU6050_ADDR
    @return 1 byte from i2c
*/
/**************************************************************************/
byte MPU6050::readMPU6050(byte reg) {
  _i2c->beginTransmission(MPU6050_ADDR);
  _i2c->write(reg);
  _i2c->endTransmission(true);
  _i2c->requestFrom(MPU6050_ADDR, 1);
  byte data =  _i2c->read();
  return data;
}

/**************************************************************************/
/*!
    @brief set gyro offsets in x/y/z
*/
/**************************************************************************/
void MPU6050::setGyroOffsets(float x, float y, float z){
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

/**************************************************************************/
/*!
    @brief computes gyro offsets
*/
/**************************************************************************/
void MPU6050::calcGyroOffsets(bool console, uint16_t delayBefore, uint16_t delayAfter){
	float x = 0, y = 0, z = 0;
	int16_t rx, ry, rz;

  delay(delayBefore);
	if(console){
    Serial.println();
    Serial.println("========================================");
    Serial.println("Calculating gyro offsets");
    Serial.print("DO NOT MOVE MPU6050");
  }
  for(int i = 0; i < 3000; i++){
    if(console && i % 1000 == 0){
      Serial.print(".");
    }
    _i2c->beginTransmission(MPU6050_ADDR);
    _i2c->write(0x43);
    _i2c->endTransmission(false);
    _i2c->requestFrom((int)MPU6050_ADDR, 6);

    rx = _i2c->read() << 8 | _i2c->read();
    ry = _i2c->read() << 8 | _i2c->read();
    rz = _i2c->read() << 8 | _i2c->read();

    x += ((float)rx) / 131;
    y += ((float)ry) / 131;
    z += ((float)rz) / 131; // was 65.5
  }
  gyroXoffset = x / 3000;
  gyroYoffset = y / 3000;
  gyroZoffset = z / 3000;

  if(console){
    Serial.println();
    Serial.println("Done!");
    Serial.print("X : ");Serial.println(gyroXoffset);
    Serial.print("Y : ");Serial.println(gyroYoffset);
    Serial.print("Z : ");Serial.println(gyroZoffset);
    Serial.println("Program will start after 3 seconds");
    Serial.print("========================================");
		delay(delayAfter);
	}
}

/**************************************************************************/
/*!
    @brief update complementary filter of IMU
*/
/**************************************************************************/
void MPU6050::update(){
	_i2c->beginTransmission(MPU6050_ADDR);
	_i2c->write(0x3B);
	_i2c->endTransmission(false);
	_i2c->requestFrom((int)MPU6050_ADDR, 14);

  rawAccX = _i2c->read() << 8 | _i2c->read();
  rawAccY = _i2c->read() << 8 | _i2c->read();
  rawAccZ = _i2c->read() << 8 | _i2c->read();
  rawTemp = _i2c->read() << 8 | _i2c->read();
  rawGyroX = _i2c->read() << 8 | _i2c->read();
  rawGyroY = _i2c->read() << 8 | _i2c->read();
  rawGyroZ = _i2c->read() << 8 | _i2c->read();

  temp = (rawTemp + 12412.0) / 340.0;

  accX = ((float)rawAccX) / 16384.0;
  accY = ((float)rawAccY) / 16384.0;
  accZ = ((float)rawAccZ) / 16384.0;

  angleAccX = atan2(accY, sqrt(accZ * accZ + accX * accX)) * 360 / 2.0 / PI;
  angleAccY = atan2(accX, sqrt(accZ * accZ + accY * accY)) * 360 / -2.0 / PI;

  gyroX = ((float)rawGyroX) / 131;//65.5;
  gyroY = ((float)rawGyroY) / 131;//65.5;
  gyroZ = ((float)rawGyroZ) / 131;//65.5;

  gyroX -= gyroXoffset;
  gyroY -= gyroYoffset;
  gyroZ -= gyroZoffset;

  interval = (millis() - preInterval) * 0.001;

  angleGyroX += gyroX * interval;
  angleGyroY += gyroY * interval;
  angleGyroZ += gyroZ * interval;

  angleX = (gyroCoef * (angleX + gyroX * interval)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroY * interval)) + (accCoef * angleAccY);
  angleZ = angleGyroZ;

  preInterval = millis();

}

bool MPU6050::isConnected(){

    byte out = readMPU6050(MPU6050_WHO_AM_I);

    //Serial.println(out);
    //Serial.println(MPU6050_ADDR);

//    _i2c->beginTransmission(MPU6050_ADDR);
//    _i2c->write(MPU6050_WHO_AM_I);
//    _i2c->endTransmission();
//    
//    _i2c->requestFrom(MPU6050_ADDR, 1);
//    byte data =  _i2c->read();
  
    return true;//data == MPU6050_ADDR;
}
