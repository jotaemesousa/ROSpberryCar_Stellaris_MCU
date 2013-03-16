/*
 * car.h
 *
 *  Created on: 22 de Dez de 2012
 *      Author: Jo�o
 */

#ifndef CAR_H_
#define CAR_H_

//#include "msp430g2553.h"


// Steering
#define MAX_STEER_RIGHT		170
#define MIN_STEER_LEFT		10
#define STEER_CENTER		90
#define U_THRESHOLD			2000
#define POS_THRESHOLD		4
#define STEER_ADC			0
#define BATTERY_ADC			1

#define MAX_PWM_STEER		100
#define MAX_PWM_DRIVE		100


struct car_steer
{
	int set_point;
	long int uc;
	int error[3];
	char new_pos;
	int current_pos;
};

extern struct car_steer ferrari_steer_pid;

void configure_pwm(void);
void default_rc_steer(void);
int pid_control(void);
void new_set_point(int );

#endif /* CAR_H_ */
