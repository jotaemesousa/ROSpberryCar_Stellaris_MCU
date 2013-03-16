/*
 * car.cpp
 *
 *  Created on: 22 de Dez de 2012
 *      Author: Jo�o
 */
#include "car.h"
#include "stdio.h"
#include <stdlib.h>
#include <string.h>

struct car_steer ferrari_steer_pid;

void configure_pwm(void)
{
	// Drive
	/*
	DRIVE_SEL |= PWM_FWD_PIN + PWM_REV_PIN;		// P2.2 and P2.5 options select
	TA1CCR0 = 999;        						// PWM period
	TA1CCTL1 = OUTMOD_6;  						// CCR2 reset/set
	PWM_FWD = 0;       							// CCR1 0% PWM duty cycle
	TA1CCTL2 = OUTMOD_6;  						// CCR2 reset/set
	PWM_REV = 0;          						// CCR2 0% PWM duty cycle
	TA1CTL = TASSEL_2 + MC_1 + TACLR;       	// SMCLK, up
	DRIVE_DIR |= PWM_FWD_PIN + PWM_REV_PIN;  	// P2.2 and P2.5 output

	// Servo
	SERVO_PWM_SEL |= SERVO_PWM_PIN;
	SERVO_PWM_DIR |= SERVO_PWM_PIN;
	//SERVO_PWM_PERIOD = 1000;
	CCTL1 = OUTMOD_6;
	SERVO_PWM = 0;
	SERVO_DIR_PORT &= ~SERVO_DIR_PIN;
	SERVO_DIR_DIR |= SERVO_DIR_PIN;


	// config ADC10
	ADC10CTL0 = ADC10SHT_2 + ADC10ON; 			// ADC10ON, interrupt enabled
	ADC10CTL1 = SERVO_ADC_INPUT;            	// input A3
	ADC10AE0 |= SERVO_ADC_CH;              		// PA.3 ADC option select*/

}

void default_rc_steer(void)
{
	memset(ferrari_steer_pid.error, 0, 3);
	ferrari_steer_pid.set_point = STEER_CENTER;
	ferrari_steer_pid.new_pos = 1;
}

void new_set_point(int n_set_point)
{
	ferrari_steer_pid.set_point = n_set_point;
	ferrari_steer_pid.new_pos = 1;
}

int pid_control(void)
{
	static char error_pid = 0;

	ferrari_steer_pid.error[2] = ferrari_steer_pid.error[1];
	ferrari_steer_pid.error[1] = ferrari_steer_pid.error[0];
	ferrari_steer_pid.error[0] = ferrari_steer_pid.set_point - ferrari_steer_pid.current_pos;

	if(ferrari_steer_pid.current_pos < MIN_STEER_LEFT)
	{
		//error_pid = 1;
		//ferrari_steer_pid.set_point = MIN_STEER_LEFT;
		ferrari_steer_pid.error[0] = 0;
		ferrari_steer_pid.error[1] = 0;
		ferrari_steer_pid.error[2] = 0;
		ferrari_steer_pid.uc = 0;
	}
	else if(ferrari_steer_pid.current_pos > MAX_STEER_RIGHT)
	{
		//error_pid = 2;
		//ferrari_steer_pid.set_point = MAX_STEER_RIGHT;
		ferrari_steer_pid.error[0] = 0;
		ferrari_steer_pid.error[1] = 0;
		ferrari_steer_pid.error[2] = 0;
		ferrari_steer_pid.uc = 0;
	}
	else
	{
		if(!error_pid)
		{
			if(ferrari_steer_pid.error[0] < -250)
			{
				ferrari_steer_pid.error[0] = -250;
			}
			else if(ferrari_steer_pid.error[0] > 250)
			{
				ferrari_steer_pid.error[0] = 250;
			}

			if(ferrari_steer_pid.new_pos)
			{
				int td = 1000;

				int Kp = 10;
				int Ki = 30;
				int Kd = 0;

				ferrari_steer_pid.uc = ferrari_steer_pid.uc + Kp * (ferrari_steer_pid.error[0] - ferrari_steer_pid.error[1]);
								+ Ki * ferrari_steer_pid.error[0] / td
								+ Kd * td * (ferrari_steer_pid.error[0] - 2 * ferrari_steer_pid.error[1] + ferrari_steer_pid.error[2]);
				if (ferrari_steer_pid.uc < -U_THRESHOLD)
				{
					ferrari_steer_pid.uc = -U_THRESHOLD;
				}
				else if(ferrari_steer_pid.uc > U_THRESHOLD)
				{
					ferrari_steer_pid.uc = U_THRESHOLD;
				}
			}

			if(ferrari_steer_pid.error[0] < POS_THRESHOLD && ferrari_steer_pid.error[0] > -POS_THRESHOLD)
			{
				ferrari_steer_pid.uc = 0;
				ferrari_steer_pid.error[0] = 0;
				ferrari_steer_pid.error[1] = 0;
				ferrari_steer_pid.new_pos = 0;
			}
		}
	}

//	if(error_pid)
//	{
//		unsigned int vals[2];
//		startConversion0(vals);
//
//		if(vals[STEER_ADC] > MIN_STEER_LEFT && vals[STEER_ADC] < MAX_STEER_RIGHT)
//		{
//			error_pid = 0;
//		}
//
//	}

	return ferrari_steer_pid.uc;

}


