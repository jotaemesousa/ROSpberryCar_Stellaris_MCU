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
#include "stdint.h"
#include "../soft_pwm.h"

//62500 count values
uint32_t beggining[2];
uint32_t end[2];

void servo_init()
{
	beggining[0] = (getSoftPWMPeriod(SERVO_ESC_PWM_GENERATOR) *
			getFreqGenerator(SERVO_ESC_PWM_GENERATOR) * SERVO_MIN_WIDTH) / 10000;
	end[0] = (getSoftPWMPeriod(SERVO_ESC_PWM_GENERATOR) *
			getFreqGenerator(SERVO_ESC_PWM_GENERATOR) * SERVO_MAX_WIDTH) / 10000;
	beggining[1] = (getSoftPWMPeriod(SERVO_ESC_PWM_GENERATOR) *
			getFreqGenerator(SERVO_ESC_PWM_GENERATOR) * ESC_MIN_WIDTH) / 10000;
	end[1] = (getSoftPWMPeriod(SERVO_ESC_PWM_GENERATOR) *
			getFreqGenerator(SERVO_ESC_PWM_GENERATOR) * ESC_MAX_WIDTH) / 10000;
}

void servo_setPosition(int position)
{
	unsigned long int value;

	if (position >= SERVO_MIN_ANGLE && position <= SERVO_MAX_ANGLE)
	{
		value = (position - SERVO_MIN_ANGLE) * (end[0] - beggining[0]) /
				(SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) + beggining[0];
		setSoftPWMDuty(SERVO_PWM, value);
	}
}

void esc_setPosition(int position)
{
	unsigned long int value;

	if (position >= ESC_MIN_ANGLE && position <= ESC_MAX_ANGLE)
	{
		value = (position - ESC_MIN_ANGLE) * (end[1] - beggining[1]) /
				(ESC_MAX_ANGLE - ESC_MIN_ANGLE) + beggining[1];
		setSoftPWMDuty(ESC_PWM, value);
	}
}
