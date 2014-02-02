/*
 b * spi.c
 *
 *  Created on: Apr 21, 2013
 *      Author: bgouveia
 */

#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <driverlib/debug.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/gpio.h>
#include <driverlib/ssi.h>

#include "spi.h"

#define CS_PIN_BASE GPIO_PORTA_BASE
#define CS_PIN GPIO_PIN_3

#define CE_PIN_BASE GPIO_PORTC_BASE
#define CE_PIN GPIO_PIN_5

static unsigned long ulClockMS=0;

void spi_init(unsigned long bitrate,unsigned long datawidth){
	//SSI
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

	//CS Pin
	MAP_GPIOPinTypeGPIOOutput(CS_PIN_BASE,CS_PIN);

	//CE Pin
	MAP_GPIOPinTypeGPIOOutput(CE_PIN_BASE,CE_PIN);

	/* Configure the SSI0 port */
	MAP_SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER, bitrate, datawidth);
	/* Configure the appropriate pins to be SSI instead of GPIO */
	MAP_GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 |GPIO_PIN_4 | GPIO_PIN_5);
	MAP_SSIEnable(SSI0_BASE);

	ulClockMS = MAP_SysCtlClockGet() / (3 * 1000);
}

void spi_cs_low()
{
	//HWREG(CS_PIN_BASE + GPIO_O_DATA + (CS_PIN << 2)) = 0;
	MAP_GPIOPinWrite(CS_PIN_BASE,CS_PIN,0x00);

}

void spi_cs_high()
{
	//HWREG(CS_PIN_BASE + GPIO_O_DATA + (CS_PIN << 2)) = CS_PIN;
	MAP_GPIOPinWrite(CS_PIN_BASE,CS_PIN,CS_PIN);
}

void spi_ce_low()
{
	//HWREG(CE_PIN_BASE + GPIO_O_DATA + (CE_PIN << 2)) = 0;
	MAP_GPIOPinWrite(CE_PIN_BASE,CE_PIN,0x00);

}

void spi_ce_high()
{
	//HWREG(CE_PIN_BASE + GPIO_O_DATA + (CE_PIN << 2)) = CE_PIN;
	MAP_GPIOPinWrite(CE_PIN_BASE,CE_PIN,CE_PIN);
}


uint8_t spi_transferByte(uint8_t data){


	unsigned long rxdata;
	MAP_SSIDataPut(SSI0_BASE,data);
	MAP_SSIDataGet(SSI0_BASE,&rxdata);

	return (rxdata&0xFF);
}

void delay(unsigned long msec){
	MAP_SysCtlDelay(ulClockMS*msec);
}

void delayMicroseconds(unsigned long usec){
	MAP_SysCtlDelay((ulClockMS/1000)*usec);
}
