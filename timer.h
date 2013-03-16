/*
 * timer.h
 *
 *  Created on: Feb 3, 2013
 *      Author: joao
 */

#ifndef TIMER_H_
#define TIMER_H_

struct timer_stellaris
{
	unsigned long int ms;
	unsigned long int s;
};

void default_timer(void);
unsigned long int millis(void);

extern struct timer_stellaris timer0;
extern void drive_pwm(void);


#endif /* TIMER_H_ */
