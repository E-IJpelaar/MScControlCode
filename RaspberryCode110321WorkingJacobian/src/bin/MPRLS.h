#ifndef MPRLS_H
#define MPRLS_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "wiringPi/wiringPi.h"
#include "wiringPi/wiringPiI2C.h"

#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>


#define RESET   -1
#define EOC     -1
#define PSI_min 0
#define PSI_max 25

#define MPRLS_DEFAULT_ADDR   (0x18)
#define MPRLS_STATUS_POWERED (0x40) 
#define MPRLS_STATUS_BUSY    (0x20)    
#define MPRLS_STATUS_FAILED  (0x04)  
#define MPRLS_STATUS_MATHSAT (0x01) 
#define MPRLS_STATUS_MASK                                                      \
  (0b01100101) ///< Sensor status mask: only these bits are set

class MPRLS {
  public:

  uint8_t lastStatus;
  
  int fd;
  float Press;

  bool begin();
  void readData();
  uint8_t readStatus(void);
  float readPressure(); // added function

};

#endif
