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

void configureGPIO(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	MAP_GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// PE0 = Alert ina226
	// PE1 = IRQ MPU6050

	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);

}

void drive_pwm(int pwm, bool brake)
{
	if(!brake)
	{
		//write pwm vales
		if (pwm==0)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, false);
		}
		else if(pwm > 0 && pwm < 127)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, false);
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2) * pwm / 127);

		}
		else if (pwm >=127)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, false);
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
		}
		else if ( pwm > -127)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, false);
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2) * -pwm / 127);
		}
		else
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, false);
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
		}
	}
	else
	{
		//write pwm vales
		if (pwm==0)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
		}
		else if(pwm > 0 && pwm < 127)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2) * (127 - pwm) / 127);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));

		}
		else if (pwm >= 127)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, true);
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, false);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
		}
		else if ( pwm > -127)
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT | PWM_OUT_5_BIT, true);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_4, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2) * (127 + pwm) / 127);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
		}
		else
		{
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_5_BIT, true);
			MAP_PWMOutputState(PWM_BASE, PWM_OUT_4_BIT, false);
			MAP_PWMPulseWidthSet(PWM_BASE, PWM_OUT_5, MAP_PWMGenPeriodGet(PWM_BASE, PWM_GEN_2));
		}
	}
}

void updateLights(RC_remote &in)
{
	static uint8_t last_buttons = 0, front_state = 0, tail_state = 0;

	if((in.buttons & R2_BUTTON) == R2_BUTTON)
	{
		if((last_buttons & R2_BUTTON) != R2_BUTTON)
		{
			front_state ++;
			tail_state ++;
			if(front_state > 3) front_state = 3;
			if(tail_state > 3) tail_state = 3;
#ifdef DEBUG
			UARTprintf("R2\n");
#endif
		}
	}
	else if((in.buttons & L2_BUTTON) == L2_BUTTON)
	{
		if((last_buttons & L2_BUTTON) != L2_BUTTON)
		{
			front_state --;
			tail_state--;
			if(front_state < 0) front_state = 0;
			if(tail_state < 0) tail_state = 0;
#ifdef DEBUG
			UARTprintf("L2\n");
#endif
		}
	}

	switch (front_state)
	{
	case 0:
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6, 0);
		break;

	case 1:
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) / 6);
		break;

	case 2:
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) / 2);
		break;

	case 3:
		PWMPulseWidthSet(PWM_BASE, PWM_OUT_6, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3)-5);
		break;
	}

	switch(tail_state)
	{
	case 0:
		if((in.buttons & R1_BUTTON) == R1_BUTTON)
		{
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) - 5);
		}
		else
		{
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, 0);
		}
		break;
	case 1:
	case 2:
	case 3:

		if((in.buttons & R1_BUTTON) == R1_BUTTON)
		{
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) - 5);
		}
		else
		{
			PWMPulseWidthSet(PWM_BASE, PWM_OUT_7, PWMGenPeriodGet(PWM_BASE, PWM_GEN_3) / 3);
		}
		break;

	}

	last_buttons = in.buttons;

}

