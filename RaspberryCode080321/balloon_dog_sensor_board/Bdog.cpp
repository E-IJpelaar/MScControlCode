#include "Bdog.h"


/**************************************************************************/
/*!
    @brief contruct class
*/
/**************************************************************************/
Bdog::Bdog() {
  ActiveDevices = 0;
}


/**************************************************************************/
/*!
    @brief contruct class
*/
/**************************************************************************/
boolean Bdog::begin() {

  Wire.begin();
  Serial.begin(115200);
  delay(100); //startup delay
  tcaselect(5);
  if (!mpu.begin()) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
  }

  if (!mpr.begin()) {
    Serial.println("Could not find a valid MPRLS sensor, check wiring!");
  }


  tcaselect(6);
  Serial.println("Initialize Pixy");
  pixy.init();
  Serial.println("Pixy Initilized");
    

  //mpu.calcGyroOffsets(true);

}


/**************************************************************************/
/*!
    @brief contruct class
*/
/**************************************************************************/
void Bdog::tcaselect(uint8_t i) {

  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();

}


/**************************************************************************/
/*!
    @brief scan ports on the i2c multiplexer
*/
/**************************************************************************/
void Bdog::tcascan() {

 // Serial.println("BDOG: starting tca-port scanning...");
  delay(10); // small delay

  uint8_t ID = 0;
  uint8_t j = 0;
  uint8_t data;
  bool NewPort = true;

  for (uint8_t t = 0; t < 8; t++) {

    NewPort = true;
    tcaselect(t); // select tca channel

    // loop over all possible i2c channels
    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == TCAADDR) continue;
      if (addr == 0x0) continue;
      if (addr == 0x60) continue;

      if (! twi_writeTo(addr, &data, 0, 1, 1)) {

        if (NewPort) {
          //Serial.print("\n TCA: port #");
          //Serial.print(t);
          NewPort = false;
        }

        Port[j]   = ID;
        Port[j + 1] = t;

        //Serial.print("\n\t ID=");
        //Serial.print(ID);

        //Serial.print(" -> 0x");
        //Serial.print(addr, HEX);

        if (addr == MPRADDR) {
          Serial.print(" = MPRLS sensor");
          Port[j + 2] = MPRADDR;
        }
        else if (addr == MPUADDR) {
          Serial.print(" = MPU6050/6500 sensor");
          Port[j + 2] = MPUADDR;
        }
        else if (addr == PIXADDR) {
         Serial.print(" = PIXY v2 Vision system");
          Port[j + 2] = PIXADDR;
        }
        else {
          //Serial.print(" = unknown device?");
        }

        ActiveDevices++;
        ID++;
        j += 3;
      }
    }
  }

  //Serial.println("");
}

/**************************************************************************/
/*!
    @brief scan ports on the i2c multiplexer
*/
/**************************************************************************/
void Bdog::showPorts() {

  uint8_t portID;
  uint8_t j = 0;

  for (int i = 0; i < ActiveDevices; i++) {

    portID = Port[j];

    Serial.print("\t ID=");
    Serial.print(portID);

    Serial.print(" tca-port=#");
    Serial.print(Port[j + 1]);

    Serial.print(" -> 0x");
    Serial.println(Port[j + 2], HEX);

    j += 3;
  }
}

/**************************************************************************/
/*!
     @return devices i2c adress
*/
/**************************************************************************/
uint8_t Bdog::getDeviceAdress(uint8_t id) {
  return Port[3 * id + 2];
}

/**************************************************************************/
/*!
    @return devices port
*/
/**************************************************************************/
uint8_t Bdog::getDevicePort(uint8_t id) {
  return Port[3 * id + 1];
}

/**************************************************************************/
/*!
    @return devices port
*/
/**************************************************************************/
float Bdog::readRegister(uint8_t id,int num) {
  uint8_t _i2c_adr;
  uint8_t _tca_prt;

  _i2c_adr = getDeviceAdress(id);
  _tca_prt = getDevicePort(id);

  tcaselect(_tca_prt);

  if (_i2c_adr == MPUADDR) {
    mpu.update();
    return mpu.getAngleX();
  }
  else if (_i2c_adr == MPRADDR) {
    mpr_data =  mpr.readPressure();
    if (isnan(mpr_data)){
      return mpr_data_;
    }
    //else if (mpr_data == 0){
    //  return mpr_data_;
    //}
    else{
      mpr_data_ = mpr_data;
      return mpr_data;
      }
  }
  else if(_i2c_adr == PIXADDR && num == 1){
      
     pixy.ccc.getBlocks();  
     pix_x =  float(pixy.ccc.blocks[0].m_x);   
     return pix_x;
     }
  else if(_i2c_adr == PIXADDR && num == 2){
      
     pixy.ccc.getBlocks();  
     pix_y =  float(pixy.ccc.blocks[0].m_y);   
     return pix_y;
     }

  else {
    return -1;
  }

}
