

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
}





/**************************************************************************/
/*!
     @brief loop
*/
/**************************************************************************/

void loop(){

  float pres  = brd.readRegister(0,1);
  float angle = brd.readRegister(1,1);
  float x_data = brd.readRegister(2,1);
  float y_data = brd.readRegister(2,2);




  

if (Serial.available() > 0) {
	  PiData = Serial.read();
     if (PiData == '1') {
      Serial.println(pres, 2);
	      }
  else if (PiData == '2') {
      Serial.println(angle,2);
    }
     if (PiData == '3'){
     Serial.println(x_data,0);
    }
    else if (PiData == '4'){
   Serial.println(y_data,0);
   }
}



  //delay(5);
}
