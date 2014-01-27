/*
 * servo.c
 *
 *  Created on: Oct 15, 2012
 *      Author: bgouveia
 */

#include "servo.h"
#include <inc/lm3s2776.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <driverlib/debug.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/gpio.h>
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "soft_pwm.h"

//62500 count values

void servo_init()
{
	setPWMGenFreq(2,50);
}

#define BASE	14
#define END  	47

void servo_setPosition(int position)
{
	unsigned long int value;

		if (position >= 0 && position <= 180)
		{
			value =  BASE + (position * (END-BASE)/180);
			setSoftPWMDuty(2, value);
		}
}
