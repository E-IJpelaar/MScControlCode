#ifndef BDOG_H
#define BDOG_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Wire.h>


#include "MPU6050.h"
#include "MPRLS.h"


#include "Pixy2I2C.h"


extern "C" {
#include "utility/twi.h" 
}

#define TCAADDR 0x70
#define MPUADDR 0x68
#define MPRADDR 0x18
#define PIXADDR 0x54

class Bdog{
  public:

    Bdog();
    boolean begin();
    void tcaselect(uint8_t i);
    void tcascan();
    void showPorts();
    
    uint8_t getDeviceAdress(uint8_t id);
    uint8_t getDevicePort(uint8_t id);
    //float readRegister(uint8_t id);
    float readRegister(uint8_t id, int num);

    uint8_t ActiveDevices;
    float mpr_data, mpr_data_;
    float pix_x,pix_y; 
   
    uint8_t Port[30] = {0};
    float Data[6] = {0};
    MPU6050 mpu;
    Adafruit_MPRLS mpr;
    Pixy2I2C pixy;

  private:
  
};

#endif
