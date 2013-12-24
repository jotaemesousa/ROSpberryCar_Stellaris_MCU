/*
 * gpio_pwm_lights.cpp
 *
 *  Created on: Dec 24, 2013
 *      Author: joao
 */

#include "gpio_pwm_lights.h"

void configurePWM(void)
{
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	MAP_GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	MAP_PWMGenConfigure(PWM_BASE,PWM_GEN_2,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
	MAP_PWMGenConfigure(PWM_BASE,PWM_GEN_3,PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);

	MAP_PWMGenPeriodSet(PWM_BASE, PWM_GEN_2, MAX_PWM_DRIVE);		// Drive PWM
	MAP_PWMGenPeriodSet(PWM_BASE, PWM_GEN_3, MAX_PWM_DRIVE);		// Drive PWM

	MAP_PWMGenEnable(PWM_BASE, PWM_GEN_2);
	MAP_PWMGenEnable(PWM_BASE, PWM_GEN_3);

	MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, false);

	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4);

}
