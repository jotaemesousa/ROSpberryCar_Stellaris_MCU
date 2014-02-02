/*
 * Encoder.cpp
 *
 *  Created on: Jul 1, 2013
 *      Author: bgouveia
 */

#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/rom_map.h>
#include <driverlib/rom.h>
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
int32_t left_counter, right_counter;
int32_t left_counter_global, right_counter_global;
int32_t last_left_counter = 0, last_right_counter = 0;
int delta_left;
uint32_t last_time_velocity = 0;


void encoder_init(){

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	MAP_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOB);
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_AHB_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	MAP_GPIOIntTypeSet(GPIO_PORTB_AHB_BASE, GPIO_PIN_6 | GPIO_PIN_7,  GPIO_BOTH_EDGES);
	MAP_GPIOPinIntEnable(GPIO_PORTB_AHB_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	MAP_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOD);
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTD_AHB_BASE, GPIO_PIN_3);
	MAP_GPIOIntTypeSet(GPIO_PORTD_AHB_BASE, GPIO_PIN_3,  GPIO_BOTH_EDGES);
	MAP_GPIOPinIntEnable(GPIO_PORTD_AHB_BASE, GPIO_PIN_3);

	left_counter = 0;
	left_counter_global = 0;
	right_counter = 0;
	right_counter_global = 0;
	left_oldstate = 0;
	right_oldstate = 0;
	delta_left = 0;

	MAP_IntEnable(INT_GPIOB);
	MAP_IntEnable(INT_GPIOD);
}

void encoder_read(int32_t *left_c, int32_t *right_c, uint8_t counter)
{
	if(!counter)
	{
		*right_c = right_counter;
		*left_c = left_counter;
	}
	else
	{
		*right_c = right_counter_global;
		*left_c = left_counter_global;
	}
	//	UARTprintf("left counter (read) : %d\n", left_counter);
	//	UARTprintf("right counter (read) : %d\n", right_counter);
}

void encoder_read_reset(int32_t *left_c, int32_t *right_c, uint8_t counter)
{
	if(!counter)
	{
		*right_c = right_counter;
		*left_c = left_counter;
		right_counter = 0;
		left_counter = 0;
	}
	else
	{
		*left_c = left_counter_global;
		*right_c = right_counter_global;
		right_counter_global = 0;
		left_counter_global = 0;
	}
	//	UARTprintf("left counter (read) : %d\n", left_counter);
	//	UARTprintf("right counter (read) : %d\n", right_counter);
}

void encoder_get_velocity(int32_t *left_vel, int32_t *right_vel, uint32_t time)
{
	*left_vel = (left_counter - last_left_counter) * 1;// / (int32_t)(time - last_time_velocity);
	*right_vel = (right_counter - last_right_counter) * 1;// / (int32_t)(time - last_time_velocity);

	last_left_counter = left_counter;
	last_right_counter = right_counter;
	last_time_velocity = time;
}

#ifdef __cplusplus
extern "C"
{
#endif
void PORTBIntHandler ()
{
	MAP_GPIOPinIntClear(GPIO_PORTB_AHB_BASE, GPIO_PIN_6 | GPIO_PIN_7);
	unsigned int currentstate = GPIOPinRead(GPIO_PORTB_AHB_BASE,GPIO_PIN_6 | GPIO_PIN_7) >> 6;
	delta_left = lookuptable[(left_oldstate << 2)|currentstate];
	left_counter += lookuptable[(left_oldstate << 2)|currentstate];
	left_counter_global += delta_left;
	left_oldstate = currentstate;
}

void PORTDIntHandler ()
{
	MAP_GPIOPinIntClear(GPIO_PORTD_AHB_BASE, GPIO_PIN_3);
	unsigned int currentstate = GPIOPinRead(GPIO_PORTD_AHB_BASE,GPIO_PIN_3) >> 3;
	right_counter += delta_left;
	right_counter_global += delta_left;
	right_oldstate = currentstate;
}
#ifdef __cplusplus
}
#endif
