#include "Wire.h"

TwoWire::TwoWire()
{
}

bool TwoWire::begin()
{
	filename = (char*)"/dev/i2c-1";
	i2cFile = open(filename, O_RDWR);
	if (i2cFile < 0)
		return false;
	return true;
}

bool TwoWire::beginTransmission(uint8_t addr)
{
	slave_address = addr;
	transmitting = true;
	if (ioctl(i2cFile, I2C_SLAVE, slave_address) < 0)
		return false;
	return true;
}

bool TwoWire::beginTransmission(int address)
{
	return beginTransmission((uint8_t)address);
}

void TwoWire::endTransmission()
{
	transmitting = false;
}

uint8_t TwoWire::write(uint8_t data)
{
	if (!tranmsitting || write(i2cFile, data, 1) != 1)
		return 0;
	return 1;
}

uint8_t TwoWrite::write(uint8_t *data, uint8_t numBytes)
{
	if (transmitting) {
		for (int i = 0; i < numBytes; ++i) {
			if (!write(data[i]))
				return i;
		}
		return numBytes;
	}
	return 0;
}

uint8_t TwoWire::requestFrom(uint8_t addr, uint8_t numBytes)
{
	bufferLength = numBytes;
	bufferBytesRead = 0;
	beginTransmission(addr);
	if (read(i2cFile, &buffer[0], numBytes) != numBytes)
		return 0;
	return numBytes;
}

uint8_t TwoWire::requestFrom(int addr, int numBytes)
{
	return requestFrom((uint8_t)addr, (uint8_t)numBytes);
}

uint8_t TwoWire::requestFrom(int numBytes)
{
	return requestFrom(slave_address, (uint8_t)numBytes);
}

uint8_t TwoWire::requestFrom(uint8_t numBytes)
{
	return requestFrom(slave_address, numBytes);
}

uint8_t TwoWire::read()
{
	bufferBytesRead++;
	if(bufferBytesRead <= bufferLength)
		return buffer[bufferBytesRead - 1];
	return 0;
}

uint8_t TwoWire::available()
{
	return bufferLength - bufferBytesRead;
}

TwoWire Wire = TwoWire();