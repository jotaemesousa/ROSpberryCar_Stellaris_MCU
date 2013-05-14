/*
 * INA226.cpp
 *
 *  Created on: May 13, 2013
 *      Author: joao
 */

#include "INA226.h"
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

INA226::INA226() {
	// TODO Auto-generated constructor stub

}

INA226::INA226(char addr)
{
	addr_ = addr;

}

INA226::~INA226() {
	// TODO Auto-generated destructor stub
}

uint16_t INA226::read_register(uint8_t reg)
{
	uint16_t conf_reg = 0;
	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, addr_ ,false);
	// true = the I2C Master is initiating a read from the slave,
	// false = The I2C Master is initiating a write to the slave

	I2CMasterDataPut(I2C0_MASTER_BASE, reg);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_SINGLE_SEND);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, addr_ ,true);
	// true = the I2C Master is initiating a read from the slave,
	// false = The I2C Master is initiating a write to the slave

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	conf_reg = I2CMasterDataGet(I2C0_MASTER_BASE) << 8;

	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	conf_reg |= I2CMasterDataGet(I2C0_MASTER_BASE);

	conf_reg_ = conf_reg;

	return conf_reg;
}

void INA226::write_register(uint8_t reg, uint16_t value)
{
	I2CMasterSlaveAddrSet(I2C0_MASTER_BASE, addr_ ,false);

	//Set reg
	I2CMasterDataPut(I2C0_MASTER_BASE, reg);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	//send value (first 8 bits)
	I2CMasterDataPut(I2C0_MASTER_BASE, (value & 0xFF00) >> 8);
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
	while(I2CMasterBusy(I2C0_MASTER_BASE));

	//send value (first 8 bits)
	I2CMasterDataPut(I2C0_MASTER_BASE, (value & 0x00FF));
	I2CMasterControl(I2C0_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
	while(I2CMasterBusy(I2C0_MASTER_BASE));
}

void INA226::set_sample_average(int avg)
{
	uint16_t new_conf_reg = conf_reg_ & 0b1111000111111111;
	switch (avg)
	{
	case 4:
		new_conf_reg |= 0b1111001111111111;
		break;
	case 16:
		new_conf_reg |= 0b1111010111111111;
		break;
	case 64:
		new_conf_reg |= 0b1111011111111111;
		break;
	case 128:
		new_conf_reg |= 0b1111100111111111;
		break;
	case 256:
		new_conf_reg |= 0b1111101111111111;
		break;
	case 512:
		new_conf_reg |= 0b1111110111111111;
		break;
	case 1024:
		new_conf_reg |= 0b1111111111111111;
		break;
	}
	write_register(CONFGURATION_REGISTER, new_conf_reg);
	conf_reg_ = new_conf_reg;
}

void INA226::set_vbus_conv_timer(uint16_t ct)
{
	uint16_t new_conf_reg = conf_reg_ & 0b1111111000111111;

	if(ct < 8)
	{
		new_conf_reg |= (ct & 0b00000111) << 6;
	}
	write_register(CONFGURATION_REGISTER, new_conf_reg);
	conf_reg_ = new_conf_reg;
}

void INA226::set_vshunt_conv_timer(uint16_t ct)
{
	uint16_t new_conf_reg = conf_reg_ & 0b1111111111000111;

	if(ct < 8)
	{
		new_conf_reg |= (ct & 0b00000111) << 3;
	}
	write_register(CONFGURATION_REGISTER, new_conf_reg);
	conf_reg_ = new_conf_reg;
}

void INA226::set_operating_mode(uint8_t mode)
{
	uint16_t new_conf_reg = conf_reg_ & 0b1111111111111000;

	if(mode < 8)
	{
		new_conf_reg |= (mode & 0b00000111);
	}
	write_register(CONFGURATION_REGISTER, new_conf_reg);
	conf_reg_ = new_conf_reg;
}
