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


#ifdef __cplusplus
extern "C"
{
#endif

void initSoftPWM(uint32_t max_freq, uint32_t res_min);
void enablePWM(void);
uint8_t setPWMGenFreq(uint8_t generator, unsigned int freq);
void  updateSoftPWM(unsigned char index);
uint8_t setSoftPWMDuty(uint8_t pwm, unsigned long int duty);
int32_t getSoftPWMPeriod(uint8_t generator);
int32_t getSoftPWMmaxDuty(uint8_t generator);
uint32_t getFreqGenerator(uint8_t generator);

#ifdef __cplusplus
}
#endif

#endif /* SOFT_PWM_H_ */
