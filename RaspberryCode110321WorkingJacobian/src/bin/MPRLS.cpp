#include "MPRLS.h"

using namespace std;
unsigned char buffer[60] = {0};

////////////////////////////////////////////////////////////////////////
bool MPRLS::begin() {

	//wiringPiSetup();
  
	//fd = wiringPiI2CSetup(MPRLS_DEFAULT_ADDR);
	
	//if(fd < 0){
		//cout << "Error: couldn't open devices!" << endl;
		//return false;
	//}
	
	if ((fd = open("/dev/i2c-1", O_RDWR)) < 0){
		cout << "Error: couldn't open devices!" << endl;
	}
  
	if (ioctl(fd, I2C_SLAVE, MPRLS_DEFAULT_ADDR) < 0){
		cout << "Error: couldn't find device on adress!" << endl;
	}
  
  	//if (RESET != -1) {
		//pinMode(RESET, OUTPUT);
		//digitalWrite(RESET, HIGH);
		//digitalWrite(RESET, LOW);
		//delay(10);
		//digitalWrite(RESET, HIGH);
  	//}

  	//if (EOC != -1) {
		//pinMode(EOC, INPUT);
  	//}

	// start-up delay
	delay(10);
	
	//return ((readStatus() & MPRLS_STATUS_MASK) == MPRLS_STATUS_POWERED);
	return true;
  
  	//return !(stat & 0b10011110);
}

////////////////////////////////////////////////////////////////////////
void MPRLS::readData() {

	//fd = wiringPiI2CSetup(MPRLS_DEFAULT_ADDR);
	//unsigned char data[4] = {0};

	//data[0] = MPRLS_DEFAULT_ADDR;
	//data[1] = 0xAA;
	//data[2] = 0;
	//data[3] = 0;
	
	//if (write(fd, data, 4) != 4 ){
		//cout << "Error: Writing error" << endl;
		//exit(-1);
	//}
	
	wiringPiI2CWriteReg8(fd, MPRLS_DEFAULT_ADDR, 0xAA);
	wiringPiI2CWriteReg8(fd, MPRLS_DEFAULT_ADDR+1, 0);
	wiringPiI2CWriteReg8(fd, MPRLS_DEFAULT_ADDR+2, 0);
	
	//wiringPiI2CWriteReg8(fd, 0x30, 0xAA);
	//wiringPiI2CWriteReg8(fd, 0x30, 0x0);
	//wiringPiI2CWriteReg8(fd, 0x30, 0x0);

	// Use the gpio to tell end of conversion
	if (EOC != -1) {
		while (!digitalRead(EOC)) {
		}
	}
	else {
		int N = 1;
		while (readStatus() & MPRLS_STATUS_BUSY) {
			//cout << "stat:= " << hex << lastStatus << " " << MPRLS_STATUS_BUSY << endl;
			cout << hex << (lastStatus) << endl;
			//cout << hex << lastStatus << endl;
			delay(10); 
		}
	}
	
	//delay(10);

	short rawPress;// was short
	uint8_t msb = wiringPiI2CReadReg8(fd, MPRLS_DEFAULT_ADDR);

	//if (msb & MPRLS_STATUS_MATHSAT) {
	//	rawPress = 0xFFFFFFFF;
	//}

	//if (msb & MPRLS_STATUS_FAILED) {
	//	rawPress = 0xFFFFFFFF;
	//}

	//if (rawPress != 0xFFFFFFFF) {
		msb = wiringPiI2CReadReg8(fd, MPRLS_DEFAULT_ADDR + 1);
		uint8_t lsb  = wiringPiI2CReadReg8(fd, MPRLS_DEFAULT_ADDR + 2);
		uint8_t lsb2 = wiringPiI2CReadReg8(fd, MPRLS_DEFAULT_ADDR + 3);
		
		cout << hex << (msb) << endl;
		cout << hex << (lsb) << endl;
		cout << hex << (lsb2) << endl;
		//rawPress = msb;
		//rawPress <<= 16;
		//rawPress |= lsb;
		//rawPress <<= 8;
		//rawPress |= lsb2;
		rawPress = (msb << 16) | (lsb << 8) | lsb2;
		
		//rawPress = msb << 8 | lsb;

		//msb = wiringPiI2CReadReg8(fd, MPRLS_DEFAULT_ADDR+3);
		//lsb = wiringPiI2CReadReg8(fd, MPRLS_DEFAULT_ADDR+4);
		//rawPress = msb << 8 | lsb;

		float tmp = (rawPress - 0x19999A) * (PSI_min - PSI_min);
		tmp /= (float)(0xE66666 - 0x19999A);
		tmp += PSI_min;

		Press =  tmp * 68.947572932;
	//}
	//else{
	//	Press = NAN;
	//}
	
	cout << "Press:= " << Press << endl;
}

////////////////////////////////////////////////////////////////////////
uint8_t MPRLS::readStatus(void) {
	
	lastStatus = wiringPiI2CRead(fd);
	
	//cout << lastStatus << endl;
	
	return lastStatus;
}

////////////////////////////////////////////////////////////////////////
float MPRLS::readPressure(){
	readData();
	return Press;	
}	
