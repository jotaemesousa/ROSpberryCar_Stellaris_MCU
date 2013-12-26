/*
 * servo.c
 *
 *  Created on: Oct 15, 2012
 *      Author: bgouveia
 */

#include "servo.h"

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

void servo_setPosition(int32_t position)
{
	uint32_t value;

	if (position >= 0 && position <= 180)
	{
		value =  BASE + (position * ((END-BASE)/180));
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, value);
	}

}

int32_t servo_valueMappedToAngle(int8_t value)
{
	if(value <= -127)
	{
		return SERVO_LEFT_ANGLE;
	}
	else if(value >= 127)
	{
		return SERVO_RIGHT_ANGLE;
	}
	else if(value < 0 && value > -127)
	{
		return (-value * (SERVO_LEFT_ANGLE - SERVO_CENTER_ANGLE))/(127) + SERVO_CENTER_ANGLE;
	}
	else if(value > 0 && value < 127)
	{
		return ((value * (SERVO_RIGHT_ANGLE - SERVO_CENTER_ANGLE))/127) + SERVO_CENTER_ANGLE;
	}
	return SERVO_CENTER_ANGLE;
}
