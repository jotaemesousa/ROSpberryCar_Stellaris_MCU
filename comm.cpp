/*
 * comm.cpp
 *
 *  Created on: Nov 23, 2013
 *      Author: joao
 */
#include "comm.h"

extern INA226 power_meter;

ROSCASDataFromRASPI struct_to_receive;
ROSCASDataToRASPI struct_to_send;
static uint32_t last_comm_millis = 0;

void initSerialComm(unsigned long ulBaud)
{
	//
	// Enable GPIO port A which is used for UART0 pins.
	// TODO: change this to whichever GPIO port you are using.
	//
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	//
	// Select the alternate (UART) function for these pins.
	// TODO: change this to select the port/pin you are using.
	//
	MAP_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOA);
	MAP_GPIOPinTypeUART(GPIO_PORTA_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Initialize the UART for console I/O.
	//
	UARTStdioInitExpClk(0,ulBaud);

	UARTEchoSet(false);
}

void communication_update_function(void)
{
	if(millis() - last_comm_millis > DELAY_BETWEEN_MSG)
	{
		addNewLinearVelocity(0);
		drive_pwm(0,1);
		servo_setPosition(SERVO_CENTER_ANGLE);
	}

	updateCarParameters();
}
