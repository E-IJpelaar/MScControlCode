

#include "Bdog.h"
Bdog brd;
char outBuffer[7];
char PiData;


/**************************************************************************/
/*!
     @@brief setup
*/
/**************************************************************************/
void setup() {
  brd.begin();                  // start bdog.class
  delay(10);

  brd.tcascan();                // assign i2c devices to port buffer
  brd.showPorts();

  //delay(12000);
}

/**************************************************************************/
/*!
     @brief loop
*/
/**************************************************************************/
void loop() {

  float pres  = brd.readRegister(0);
  float angle = brd.readRegister(1);

  if (Serial.available() > 0) {
    PiData = Serial.read();
    if (PiData == 'p') {
      float pres  = brd.readRegister(0);
      Serial.print(pres, 2);
    }
    else if (PiData = 'a') {
      float angle = brd.readRegister(1);
      Serial.println(angle, 2);
    }
  }


  //float pres = brd.readRegister(0);
  //float angle = brd.readRegister(1);

  // float data;

  //Serial.print("<");
  //Serial.print(pres,4);
  //Serial.println(">");


  //Serial.print("+");
  //Serial.print(angle,4);
  //Serial.println("+");

  //brd.Data[0] = brd.readRegister(0);
  //brd.Data[1] = brd.readRegister(1);

  //Serial.println(brd.readRegister(0));
  //Serial.println(brd.Data[1]);

  delay(5);
}
