#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define MAX_BUFFER_SIZE 64

class TwoWire {
private:
	uint8_t slave_address;
	int i2cFile;
	char *filename;
	bool transmitting;
	uint8_t buffer[MAX_BUFFER_SIZE];
	int bufferBytesRead;
	int bufferLength;

public:
	TwoWire();
	bool begin(); //init i2c
	bool beginTransmission(uint8_t addr);// begin transission with the 7-bit slave address provided
	bool beginTransmission(int addr);
	void endTransmission(); // end any ongoing transmissions
	uint8_t write(uint8_t data);//write one byte of data to slave currently in transmission with
	uint8_t write(uint8_t *data, uint8_t numBytes);//write x bytes of data to slave currently in transmission
	uint8_t requestFrom(uint8_t addr, uint8_t numBytes);
	uint8_t requestFrom(uint8_t numBytes);
	//		uint8_t requestFrom(uint8_t addr, uint8_t numBytes, uint8_t sendstop);
	//		uint8_t requestFrom(uint8_t addr, uint8_t numBytes, uint32_t iaddr, uint8_t);
	uint8_t requestFrom(int addr, int numBytes);
	uint8_t requestFrom(int numBytes);
	//		uint8_t requestFrom(int addr, int numBytes, int sendstop);
	uint8_t read();
	//		uint8_t read(uint8_t numBytes);
	uint8_t available();
};

extern TwoWire Wire;