/*
 * timer.c
 *
 *  Created on: Feb 3, 2013
 *      Author: joao
 */

#include "timer.h"
#include "rc_cmds.h"
#include "servo.h"

struct timer_stellaris timer0;

void default_timer(void)
{
	timer0.s = 0;
	timer0.ms = 0;
}

unsigned long int millis(void)
{
	return(timer0.ms + timer0.s*1000L);
}

// This interrupt runs every ms
void SysTickHandler(void)
{
	timer0.ms++;

	if(timer0.ms >= 1000)
	{
		timer0.ms = 0;
		timer0.s++;
	}

	if(millis() - ferrari288gto.last_millis > THRESHOLD_BETWEEN_MSG)
	{
		ferrari288gto.Drive = 0;
		ferrari288gto.Steer = SERVO_CENTER_ANGLE;

		drive_pwm();

		servo_setPosition(ferrari288gto.Steer);
	}
}
