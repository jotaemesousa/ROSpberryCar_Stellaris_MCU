/*
 * Encoder.cpp
 *
 *  Created on: Jul 1, 2013
 *      Author: bgouveia
 */

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/debug.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/gpio.h>
#include <driverlib/pwm.h>
#include <driverlib/interrupt.h>
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

#include "Encoder.h"

static const int lookuptable [16] = {0,1,-1,2,-1,0,2,1,1,2,0,-1,2,-1,1,0};

unsigned int left_oldstate, right_oldstate;
int left_counter, right_counter;
int last_left_counter = 0, last_right_counter = 0;
int delta_left;
uint32_t last_time_velocity = 0;


void encoder_init(){

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7,  GPIO_BOTH_EDGES);
	GPIOPinIntEnable(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, GPIO_PIN_3);
	GPIOIntTypeSet(GPIO_PORTD_BASE, GPIO_PIN_3,  GPIO_BOTH_EDGES);
	GPIOPinIntEnable(GPIO_PORTD_BASE, GPIO_PIN_3);

	left_counter = 0;
	right_counter = 0;
	left_oldstate = 0;
	right_oldstate = 0;
	delta_left = 0;

	IntEnable(INT_GPIOB);
	IntEnable(INT_GPIOD);
}

void encoder_read(int *left_c, int *right_c)
{
	*right_c = right_counter;
	*left_c = left_counter;
//	UARTprintf("left counter (read) : %d\n", left_counter);
//	UARTprintf("right counter (read) : %d\n", right_counter);
}

void encoder_get_velocity(int16_t *left_vel, int16_t *right_vel, uint32_t time)
{
	*left_vel = (left_counter - last_left_counter) * 1000 / (int)(time - last_time_velocity);
	*right_vel = (right_counter - last_right_counter) * 1000 / (int)(time - last_time_velocity);

	last_left_counter = left_counter;
	last_right_counter = right_counter;
	last_time_velocity = time;
}

void PORTBIntHandler ()
{
	GPIOPinIntClear(GPIO_PORTB_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	unsigned int currentstate = GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_6 | GPIO_PIN_7) >> 6;
//	UARTprintf("Port B \noldstate : %x\n",left_oldstate);
//	UARTprintf("currentstate : %x\n",currentstate);
//	UARTprintf("state : %x\n",(left_oldstate << 2)|currentstate);
//	UARTprintf("counter += %d\n",lookuptable[(left_oldstate << 2)|currentstate]);
	delta_left = lookuptable[(left_oldstate << 2)|currentstate];
	left_counter += lookuptable[(left_oldstate << 2)|currentstate];
	left_oldstate = currentstate;
}

void PORTDIntHandler ()
{
	GPIOPinIntClear(GPIO_PORTD_BASE, GPIO_PIN_3);
	unsigned int currentstate = GPIOPinRead(GPIO_PORTD_BASE,GPIO_PIN_3) >> 3;
//	UARTprintf("Port D \nold counter : %d\n",right_counter);
	right_counter += delta_left;
	delta_left = 0;

	//	UARTprintf("Port D \nnew counter : %d\n",right_counter);
//	UARTprintf("currentstate : %x\n",currentstate);
//	UARTprintf("state : %x\n",(right_oldstate << 2)|currentstate);
//	UARTprintf("counter += %d\n",lookuptable[(right_oldstate << 2)|currentstate]);
//	right_counter += lookuptable[(right_oldstate << 2)|currentstate];
	right_oldstate = currentstate;
}
