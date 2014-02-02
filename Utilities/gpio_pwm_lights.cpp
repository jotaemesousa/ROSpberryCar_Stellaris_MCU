/*
 * gpio_pwm_lights.cpp
 *
 *  Created on: Dec 24, 2013
 *      Author: joao
 */

#include "gpio_pwm_lights.h"

void configurePWM(void)
{
	initSoftPWM(500,40);
	setPWMGenFreq(3,500);
	setPWMGenFreq(4,500);
}

void configureGPIO(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	// PE0 = Alert ina226
	// PE1 = IRQ MPU6050

	GPIOPinTypeGPIOInput(GPIO_PORTE_AHB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

}

void drive_pwm(int pwm, bool brake)
{
	if(!brake)
		{
			//write pwm vales
			if(pwm == 0)
			{
				setSoftPWMDuty(4,0);
				setSoftPWMDuty(5,0);
			}
			else if(pwm > 0 && pwm < 127)
			{
				setSoftPWMDuty(4,getSoftPWMmaxDuty(3) * pwm / 127);
				setSoftPWMDuty(5,0);
			}
			else if (pwm >=127)
			{
				setSoftPWMDuty(4,getSoftPWMmaxDuty(3));
				setSoftPWMDuty(5,0);
			}
			else if (pwm > -127)
			{
				setSoftPWMDuty(5,getSoftPWMmaxDuty(3) * -pwm / 127);
				setSoftPWMDuty(4,0);
			}
			else
			{
				setSoftPWMDuty(5,getSoftPWMmaxDuty(3));
				setSoftPWMDuty(4,0);
			}
		}
		else
		{
			if(pwm == 0)
			{
				setSoftPWMDuty(4,getSoftPWMmaxDuty(3));
				setSoftPWMDuty(5,getSoftPWMmaxDuty(3));
			}
			else if(pwm > 0 && pwm < 127)
			{
				setSoftPWMDuty(4,getSoftPWMmaxDuty(3));
				setSoftPWMDuty(5,getSoftPWMmaxDuty(3) * (127 - pwm) / 127);
			}
			else if (pwm >=127)
			{
				setSoftPWMDuty(4,getSoftPWMmaxDuty(3));
				setSoftPWMDuty(5,0);
			}
			else if (pwm > -127)
			{
				setSoftPWMDuty(4,getSoftPWMmaxDuty(3) * (127 - pwm) / 127);
				setSoftPWMDuty(5,getSoftPWMmaxDuty(3));
			}
			else
			{
				setSoftPWMDuty(5,getSoftPWMmaxDuty(3));
				setSoftPWMDuty(4,0);
			}
		}
}

void updateLights(RC_remote &in)
{
	static uint8_t last_buttons = 0;
	static int8_t front_state = 0, tail_state = 0;

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
			setSoftPWMDuty(6,0);
			break;

		case 1:
			setSoftPWMDuty(6,getSoftPWMmaxDuty(4) / 6);
			break;

		case 2:
			setSoftPWMDuty(6,getSoftPWMmaxDuty(4) / 3);
			break;

		case 3:
			setSoftPWMDuty(6,getSoftPWMmaxDuty(4));
			break;
		}

		switch(tail_state)
		{
		case 0:
			if((in.buttons & R1_BUTTON) == R1_BUTTON)
			{
				setSoftPWMDuty(7,getSoftPWMmaxDuty(4));
			}
			else
			{
				setSoftPWMDuty(7,0);
			}
			break;
		case 1:
		case 2:
		case 3:

			if((in.buttons & R1_BUTTON) == R1_BUTTON)
			{
				setSoftPWMDuty(6,getSoftPWMmaxDuty(4));
			}
			else
			{
				setSoftPWMDuty(6,getSoftPWMmaxDuty(4) / 3);
			}
			break;

		}

		last_buttons = in.buttons;
}

