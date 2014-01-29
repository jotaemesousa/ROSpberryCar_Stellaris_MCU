/*
 * soft_pwm.c
 *
 *  Created on: Sep 21, 2013
 *      Author: joao
 */
#include <inc/lm3s5732.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_timer.h>
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
#include <driverlib/debug.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/gpio.h>
#include <driverlib/pwm.h>
#include <driverlib/interrupt.h>
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "driverlib/timer.h"
#include "soft_pwm.h"
#include "stdint.h"
#include "stdlib.h"

#define UART_DEBUG

static const uint32_t pin_table [8] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_4, GPIO_PIN_6};
static const uint32_t pin_table_index [8] = {0, 1, 0, 1, 6, 7, 4, 6};
static const uint32_t port_table [4] = {GPIO_PORTD_AHB_BASE, GPIO_PORTB_AHB_BASE, GPIO_PORTA_AHB_BASE, GPIO_PORTC_AHB_BASE};
static const uint32_t periph_table [4] = {SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOC};

uint32_t lastDutyCycle[MAX_PWM_GENERATORS*2];


void initSoftPWM(uint32_t max_freq, uint32_t res_min)
{
	if(max_freq < 10000 && max_freq > 0 &&
			res_min > 0 && max_freq * res_min <= 500000)
	{
		max_pwm_freq = max_freq;
		min_pwm_res = res_min;
	}
	else
	{
		max_pwm_freq = 500;
		min_pwm_res = 100;
	}
#ifdef UART_DEBUG
	UARTprintf("Freq = %u, min_res = %u\n", max_pwm_freq, min_pwm_res);
#endif
	uint32_t value_timer = SysCtlClockGet() / (max_pwm_freq * min_pwm_res);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
	TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / (max_pwm_freq * min_pwm_res));

	int i;
	for(i = 0; i < MAX_PWM_GENERATORS; i++)
	{
		pwm_counters[i] = 0;
		max_count[i] = 0;
		config_done[i] = 0;
		compare_value[i] = 0;
	}
}

void enablePWM(void)
{
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	TimerEnable(TIMER0_BASE, TIMER_A);
}

uint8_t setPWMGenFreq(uint8_t generator, unsigned int freq)
{
	config_done[generator - 1] = 0;
	if(generator <= MAX_PWM_GENERATORS && generator > 0)
	{
		if(freq > 0 && freq <= max_pwm_freq)
		{
			freq_pwm[generator - 1] = freq;
			max_count[generator -1] = max_pwm_freq * min_pwm_res / freq_pwm[generator - 1];
			config_done[generator - 1] = 1;

#ifdef UART_DEBUG
			UARTprintf(" max count  %u", max_count[generator - 1] );
			UARTprintf("Freq max = %u, min_res = %u\n", max_pwm_freq, min_pwm_res);
			UARTprintf("port = %x, pin = %x\n", port_table[generator - 1], (1 << pin_table[(generator - 1) * 2]) | (1 << pin_table[(generator - 1) * 2 + 1]));
#endif
			MAP_SysCtlPeripheralEnable(periph_table[generator - 1]);
			MAP_SysCtlGPIOAHBEnable(periph_table[generator - 1]);
			MAP_GPIOPinTypeGPIOOutput(port_table[generator - 1], (pin_table[(generator - 1) * 2]) | (pin_table[(generator - 1) * 2 + 1]));

			lookUp_pwm[(generator - 1) * 2] = (uint8_t *)malloc(max_count[generator - 1]);
			lookUp_pwm[(generator - 1) * 2 + 1] = (uint8_t *)malloc(max_count[generator - 1]);

			memset(lookUp_pwm[(generator - 1) * 2], 0, max_count[generator - 1]);
			memset(lookUp_pwm[(generator - 1) * 2 + 1], 0, max_count[generator - 1]);
			//			int i = 0;
			//			for (i = 0; i < max_count[generator - 1]; ++i)
			//			{
			//				lookUp_pwm[(generator - 1) * 2][i] = 0;
			//				lookUp_pwm[(generator - 1) * 2 + 1][i] = 0;
			//			}

			lastDutyCycle[(generator - 1) * 2] = 0;
			lastDutyCycle[(generator - 1) * 2 + 1] = 0;

			//			uint8_t array[pwm_counters[(generator - 1)]];
			//			memcpy(array,lookUp_pwm[(generator - 1) * 2],pwm_counters[(generator - 1)]);

#ifdef UART_DEBUG2
			int i;
			UARTprintf("printing tab...");
			for(i = 0; i < max_count[generator - 1]; i++)
			{
				UARTprintf("%d ", lookUp_pwm[(generator - 1) * 2 ][i]);
			}
			UARTprintf("printing tab2... \n");
			for(i = 0; i < max_count[generator - 1]; i++)
			{
				UARTprintf("%d ", lookUp_pwm[(generator - 1) * 2 +1][i]);
			}
			UARTprintf("done\n");
#endif
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 2;
	}
}

void updateSoftPWM(unsigned char index)
{
	uint32_t index2 = index << 1;
	uint32_t index21 = (index << 1) + 1;
	uint32_t pin1 = pin_table[index2];
	uint32_t pin2 = pin_table[index21];
	uint32_t value1 = (lookUp_pwm[index2][pwm_counters[index]])<< pin_table_index[index2];
	uint32_t value2 = (lookUp_pwm[index21][pwm_counters[index]])<< pin_table_index[index21];

	//	uint8_t array[pwm_counters[index]];
	//	memcpy(array,lookUp_pwm[index2],pwm_counters[index]);
	uint8_t coiso = lookUp_pwm[index2][pwm_counters[index]];
	uint8_t coiso2 = lookUp_pwm[index21][pwm_counters[index]];

	HWREG(port_table[index] + GPIO_O_DATA + ((pin1 | pin2) << 2)) = (value1 | value2);  //portb7 low

	pwm_counters[index] = (pwm_counters[index] + 1) % max_count[index];

#ifdef UART_DEBUG
	//UARTprintf(" max count  %u, port %x, pin = %x , pin = %x\n", max_count[index], port_table[index], pin_table[index * 2], pin_table[index * 2 + 1]);
#endif
}

uint8_t setSoftPWMDuty(uint8_t pwm, unsigned long int duty)
{
	int32_t dcycle = duty;
	if(pwm >= 0 && pwm < (MAX_PWM_GENERATORS * 2))
	{
		if(dcycle < getSoftPWMPeriod((pwm >> 1) + 1) && dcycle >= 0)
		{
			if(dcycle < lastDutyCycle[pwm])
			{
				int32_t i;
				for(i = lastDutyCycle[pwm]; i >= dcycle; i--)
				{
					lookUp_pwm[pwm][i] = 0;
				}
			}
			else if(dcycle > lastDutyCycle[pwm])
			{
				int32_t i;
				for(i = lastDutyCycle[pwm]; i <= dcycle; i++)
				{
					lookUp_pwm[pwm][i] = 1;
				}
			}
			lastDutyCycle[pwm] = dcycle;
			return 0;
		}
		else if(dcycle >= getSoftPWMPeriod((pwm >> 1) + 1))
		{
			int32_t i;
			for(i = lastDutyCycle[pwm]; i < getSoftPWMPeriod((pwm >> 1) + 1); i++)
			{
				lookUp_pwm[pwm][i] = 1;
			}

			lastDutyCycle[pwm] = dcycle;
			return 1;
		}
		else
		{
			int32_t i;
			for(i = lastDutyCycle[pwm]; i >= 0; i--)
			{
				lookUp_pwm[pwm][i] = 0;
			}

			lastDutyCycle[pwm] = dcycle;
			//			memset(lookUp_pwm[pwm], 0, dcycle* max_count[pwm/2]);
			return 2;
		}
	}
	else
	{
		return 3;
	}
}

int32_t getSoftPWMPeriod(uint8_t generator)
{
	if(config_done[generator - 1])
	{
		return max_count[generator - 1];
	}
	else
	{
		return -1;
	}
}

int32_t getSoftPWMmaxDuty(uint8_t generator)
{
	return getSoftPWMPeriod(generator) - 1;
}

uint32_t getFreqGenerator(uint8_t generator)
{
	if(generator <= MAX_PWM_GENERATORS && generator > 0)
	{
		return freq_pwm[generator -1];
	}
	return 0;
}

void Timer0IntHandler(void)
{
	HWREG(TIMER0_BASE + TIMER_O_ICR) = TIMER_TIMA_TIMEOUT;
	//TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	//HWREG(GPIO_PORTB_AHB_BASE + GPIO_O_DATA + ((GPIO_PIN_7 ) << 2)) = (GPIO_PIN_7 );  //portb7 high
	//HWREG(GPIO_PORTD_AHB_BASE + GPIO_O_DATA + ((GPIO_PIN_0 ) << 2)) = (GPIO_PIN_0 );  //portb7 low
	unsigned char i= 0;
	updateSoftPWM(0);
	updateSoftPWM(2);
	updateSoftPWM(3);
	//	for(i = 0; i < MAX_PWM_GENERATORS; i++)
	//	{
	//		if(config_done[i])
	//		{
	//			updateSoftPWM(i);
	//
	//		}
	//	}

	//HWREG(GPIO_PORTB_AHB_BASE + GPIO_O_DATA + ((GPIO_PIN_7 ) << 2)) = (0 );  //portb7 low
	//HWREG(GPIO_PORTD_AHB_BASE + GPIO_O_DATA + ((GPIO_PIN_0 ) << 2)) = (0 );  //portb7 low
}

