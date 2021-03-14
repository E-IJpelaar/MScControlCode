

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

  float pres  = brd.readRegister(0);
  float angle = brd.readRegister(1);

if (Serial.available() > 0) {
	   PiData = Serial.read();
     if (PiData == '1') {
      Serial.println(pres, 2);
	      }
  else if (PiData == '2') {
      Serial.println(angle,2);
    }
}

//  Serial.print("<");
//  Serial.print(pres,2);
//  Serial.println(">");


  //Serial.print("+");
  //Serial.print(angle,2);
  //Serial.println("+");


//  if (Serial.available() > 0) {
//   PiData = Serial.read();
//     if (PiData == '1') {
//      Serial.print("p");
//      Serial.print(pres, 2);
//      Serial.print("a");
//      Serial.print(angle, 2);
//      Serial.println();
//        }
//}



  //delay(5);
}
