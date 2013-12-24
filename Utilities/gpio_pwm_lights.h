/*
 * gpio_pwm_lights.h
 *
 *  Created on: Dec 24, 2013
 *      Author: joao
 */

#ifndef GPIO_PWM_LIGHTS_H_
#define GPIO_PWM_LIGHTS_H_

#include "../common_includes.h"
#include "../defined_macros.h"
#include "../remote_defines.h"
#include "servo.h"

void configurePWM(void);
void configureGPIO(void);
void drive_pwm(int pwm, bool brake);
void updateLights(RC_remote &in);
bool convert_values(RC_remote &in, RC_Param &car_param, RC_Cmds &out);

#endif /* GPIO_PWM_LIGHTS_H_ */
