#ifndef I2CUTIL_H_
#define I2CUTIL_H_

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
//#include "inc/hw_ints.h"
//#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
//#include "driverlib/sysctl.h"

int i2cWrite(unsigned char address, unsigned char* bytes, int numBytes);

int i2cRead(unsigned char address, unsigned char* bytes, int numBytes);

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#endif
