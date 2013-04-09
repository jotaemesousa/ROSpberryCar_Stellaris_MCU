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
#include <driverlib/pwm.h>
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"

//62500 count values

void servo_init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
	GPIOPinTypePWM(GPIO_PORTD_BASE,GPIO_PIN_1);


	PWMGenConfigure(PWM_BASE,PWM_GEN_0,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, 62500);
	servo_setPosition(SERVO_CENTER_ANGLE);
	PWMOutputState(PWM_BASE, PWM_OUT_1_BIT, true);
	PWMGenEnable(PWM_BASE, PWM_GEN_0);

}



#define BASE 1560
#define END  7810

void servo_setPosition(int position)
{
	unsigned long value;
//	int pos_temp;
//
//	if(position_remote == 0)
//	{
//		value =  BASE + (SERVO_CENTER_ANGLE * ((END-BASE)/180));
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, value);
//	}
//	else if(position_remote < 0 && position_remote >= -100)
//	{
//		pos_temp = SERVO_CENTER_ANGLE + (SERVO_LEFT_ANGLE - SERVO_CENTER_ANGLE) * position_remote / -100;
//		value =  BASE + (pos_temp * ((END-BASE)/180));
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, value);
//	}
//	else if(position_remote > 0 && position_remote <= 100)
//	{
//		pos_temp = SERVO_CENTER_ANGLE + (SERVO_RIGHT_ANGLE - SERVO_CENTER_ANGLE) * position_remote / 100;
//		value =  BASE + (pos_temp * ((END-BASE)/180));
//		PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, value);
//	}

	if (position >= 0 && position <= 180)
	{
		value =  BASE + (position * ((END-BASE)/180));
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, value);
	}

}
