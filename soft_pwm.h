/*
 * soft_pwm.h
 *
 *  Created on: Sep 21, 2013
 *      Author: joao
 */

#ifndef SOFT_PWM_H_
#define SOFT_PWM_H_

#include "stdint.h"
#define MAX_PWM_GENERATORS		4


unsigned long int freq_pwm[MAX_PWM_GENERATORS];
unsigned long int max_pwm_freq;
unsigned long int min_pwm_res;
unsigned long int compare_value[MAX_PWM_GENERATORS*2];
unsigned long int max_count[MAX_PWM_GENERATORS];
unsigned long int pwm_counters[MAX_PWM_GENERATORS];
unsigned char config_done[MAX_PWM_GENERATORS];

uint8_t *lookUp_pwm[MAX_PWM_GENERATORS*2];
uint8_t *lookUp_pwm2[MAX_PWM_GENERATORS*2];

uint8_t **actual_buffer;
uint8_t **last_buffer;

void initSoftPWM(unsigned int max_freq, unsigned int res_min);
void enablePWM(void);
uint8_t setPWMGenFreq(uint8_t generator, unsigned int freq);
void  updateSoftPWM(unsigned char index);
uint8_t setSoftPWMDuty(uint8_t generator, unsigned long int dcycle);
int32_t getSoftPWMPeriod(uint8_t generator);

#endif /* SOFT_PWM_H_ */
