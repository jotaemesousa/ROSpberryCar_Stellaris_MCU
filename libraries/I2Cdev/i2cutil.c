#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_i2c.h>
//#include "inc/hw_ints.h"
//#include "driverlib/interrupt.h"
#include <driverlib/i2c.h>
//#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"
#include "i2cutil.h"
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>


int i2cWrite(unsigned char address, unsigned char* bytes, int numBytes)
{
    MAP_I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, address, false);  // false = write.  true = read.
    MAP_I2CMasterDataPut(I2C0_MASTER_BASE, *bytes++);

    // Send single piece of data if it's the only piece to send
    if (numBytes == 1){
    	MAP_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
        // Wait until done transmitting
        while(MAP_I2CMasterBusy(I2C0_MASTER_BASE));

        return 0; // all done
    }
    // We have multiple bytes to send
    //
    // Start sending the first byte of the burst (already loaded with I2CMasterDataPut)
    //
    MAP_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    // Wait until done transmitting
    while(MAP_I2CMasterBusy(I2C0_MASTER_BASE));

    //i2c_buffer_index--;
    //data++;
    numBytes--;

    //
    // Continue sending consecutive data
    //
    while(numBytes > 1)
    {
    	MAP_I2CMasterDataPut(I2C0_MASTER_BASE, *bytes++);
    	MAP_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        while(MAP_I2CMasterBusy(I2C0_MASTER_BASE));
        numBytes--;
    }

    //
    // Send last piece of data and a STOP
    //
    MAP_I2CMasterDataPut(I2C0_MASTER_BASE, *bytes);
    MAP_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(MAP_I2CMasterBusy(I2C0_MASTER_BASE));

    return 0;
}

int i2cRead(unsigned char address, unsigned char* bytes, int numBytes)
{

	MAP_I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, address, true);  // false = write.  true = read.

    // Send single piece of data if it's the only piece to send
    if (numBytes == 1){
    	MAP_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        // Wait until done transmitting
        while(MAP_I2CMasterBusy(I2C0_MASTER_BASE));
        *bytes++ = MAP_I2CMasterDataGet(I2C_MASTER_BASE);
        numBytes--;
        return 0; // all done
    }
    // We have multiple bytes to send
    //
    // Start sending the first byte of the burst (already loaded with I2CMasterDataPut)
    //
    MAP_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    // Wait until done transmitting
    while(MAP_I2CMasterBusy(I2C0_MASTER_BASE));

    //i2c_buffer_index--;
    //data++;
    *bytes++ = MAP_I2CMasterDataGet(I2C_MASTER_BASE);
    numBytes--;

    //
    // Continue sending consecutive data
    //
    while(numBytes > 1)
    {
    	MAP_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(MAP_I2CMasterBusy(I2C0_MASTER_BASE));
        *bytes++ = MAP_I2CMasterDataGet(I2C_MASTER_BASE);
        numBytes--;
    }

    //
    // Send last piece of data and a STOP
    //
    MAP_I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(MAP_I2CMasterBusy(I2C0_MASTER_BASE));
    *bytes++ = MAP_I2CMasterDataGet(I2C_MASTER_BASE);
    numBytes--;

    return 0;
}
