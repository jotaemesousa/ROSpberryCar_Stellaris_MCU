/*
 * adxl345.c
 *
 *  Created on: Jan 27, 2013
 *      Author: joao
 */
#include "adxl345.h"
#include <inc/lm3s1776.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <driverlib/debug.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/gpio.h>
#include <driverlib/rom.h>
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "utils/cmdline.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include <stdint.h>
#include <math.h>

void init_adlx345(void)
{
	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ADXL345_ADDR ,false);

	//Set power reg pointer
	I2CMasterDataPut(I2C0_MASTER_BASE, POWER_REGISTER);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	//send power register S&S
	I2CMasterDataPut(I2C0_MASTER_BASE, 0x08);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	//Read data format
	//ADXL345_SetRange(2,1);
	//Set data format pointer
	I2CMasterDataPut(I2C0_MASTER_BASE, DATA_FORMAT_REGISTER);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	//send power register S&S
	I2CMasterDataPut(I2C0_MASTER_BASE, 0x08);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	//Set bw rate pointer
	I2CMasterDataPut(I2C0_MASTER_BASE, BW_RATE_REGISTER);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	//send power register S&S
	I2CMasterDataPut(I2C0_MASTER_BASE, 0x0A);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

}

char get_dev_id_adxl345(void)
{
	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ADXL345_ADDR ,false);
	//send devid
	I2CMasterDataPut(I2C0_MASTER_BASE, 0x00);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ADXL345_ADDR,true);
	// true = the I2C Master is initiating a read from the slave,
	// false = The I2C Master is initiating a write to the slave
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	while(I2CMasterBusy(I2C0_MASTER_BASE))
	{
	}

	return I2CMasterDataGet(I2C0_MASTER_BASE);

}

int get_acc_x(void)
{
	int acc_x = 0;
	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ADXL345_ADDR ,false);
	// true = the I2C Master is initiating a read from the slave,
	// false = The I2C Master is initiating a write to the slave

	//Set data x pointer
	I2CMasterDataPut(I2C0_MASTER_BASE, DATA_X_LSB_REGISTER);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ADXL345_ADDR,true);
	// true = the I2C Master is initiating a read from the slave,
	// false = The I2C Master is initiating a write to the slave

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	acc_x |= I2CMasterDataGet(I2C0_MASTER_BASE);

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	acc_x |= I2CMasterDataGet(I2C0_MASTER_BASE) << 8;

	acc_x = TwoComplement2ModSig_16bit(acc_x);

	return acc_x;
}

int get_acc_y(void)
{
	int acc_y = 0;
	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ADXL345_ADDR ,false);
	// true = the I2C Master is initiating a read from the slave,
	// false = The I2C Master is initiating a write to the slave

	//Set data x pointer
	I2CMasterDataPut(I2C0_MASTER_BASE, DATA_Y_LSB_REGISTER);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ADXL345_ADDR,true);
	// true = the I2C Master is initiating a read from the slave,
	// false = The I2C Master is initiating a write to the slave

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	acc_y |= I2CMasterDataGet(I2C0_MASTER_BASE);

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	acc_y |= I2CMasterDataGet(I2C0_MASTER_BASE) << 8;

	acc_y = TwoComplement2ModSig_16bit(acc_y);

	return acc_y;
}

int get_acc_z(void)
{
	int acc_z = 0;
	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ADXL345_ADDR ,false);
	// true = the I2C Master is initiating a read from the slave,
	// false = The I2C Master is initiating a write to the slave

	//Set data x pointer
	I2CMasterDataPut(I2C0_MASTER_BASE, DATA_Z_LSB_REGISTER);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ADXL345_ADDR,true);
	// true = the I2C Master is initiating a read from the slave,
	// false = The I2C Master is initiating a write to the slave

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	acc_z |= I2CMasterDataGet(I2C0_MASTER_BASE);

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	acc_z |= I2CMasterDataGet(I2C0_MASTER_BASE) << 8;

	acc_z = TwoComplement2ModSig_16bit(acc_z);

	return acc_z;
}

void get_acc(struct Accelerometer * acc)
{
	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ADXL345_ADDR ,false);
	// true = the I2C Master is initiating a read from the slave,
	// false = The I2C Master is initiating a write to the slave

	//Set data x pointer
	I2CMasterDataPut(I2C0_MASTER_BASE, DATA_X_LSB_REGISTER);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	//receive data X, Y & Z
	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ADXL345_ADDR,true);
	// true = the I2C Master is initiating a read from the slave,
	// false = The I2C Master is initiating a write to the slave

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	acc->acc_x = 0;
	acc->acc_y = 0;
	acc->acc_z = 0;

	acc->acc_x |= I2CMasterDataGet(I2C0_MASTER_BASE);

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	acc->acc_x |= I2CMasterDataGet(I2C0_MASTER_BASE) << 8;

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	acc->acc_y |= I2CMasterDataGet(I2C0_MASTER_BASE);

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	acc->acc_y |= I2CMasterDataGet(I2C0_MASTER_BASE) << 8;

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	acc->acc_z |= I2CMasterDataGet(I2C0_MASTER_BASE);

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	acc->acc_z |= I2CMasterDataGet(I2C0_MASTER_BASE) << 8;

	acc->acc_x = TwoComplement2ModSig_16bit(acc->acc_x);
	acc->acc_y = TwoComplement2ModSig_16bit(acc->acc_y);
	acc->acc_z = TwoComplement2ModSig_16bit(acc->acc_z);

}

void ADXL345_GetRange(void)
{
	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ADXL345_ADDR ,false);
	I2CMasterDataPut(I2C0_MASTER_BASE, DATA_FORMAT_REGISTER);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, ADXL345_ADDR,true);
	// true = the I2C Master is initiating a read from the slave,
	// false = The I2C Master is initiating a write to the slave

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
	while(I2CMasterBusy(I2C0_MASTER_BASE));
	UARTprintf("data format %x\n", I2CMasterDataGet(I2C0_MASTER_BASE));
}

int TwoComplement2ModSig_16bit(uint16_t a)
{
	int mask = 0;
	mask = 0x8000;
	if((a & mask) == mask)
	{
		a = ~a;
		a = a + 1;
		return -a;
	}
	else
	{
		return a;
	}

}
