/*
 * servo.h
 *
 *  Created on: Oct 15, 2012
 *      Author: bgouveia
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "../common_includes.h"

// Servo Settings
#define SERVO_CENTER_ANGLE		97 //97
#define SERVO_LEFT_ANGLE		132.0//133
#define SERVO_RIGHT_ANGLE		55.0

#define SERVO_ESC_PWM_GENERATOR	1
#define SERVO_PWM				1
#define ESC_PWM					0

#define SERVO_MIN_WIDTH			10		//ms/10
#define SERVO_MAX_WIDTH			20		//ms/10
#define SERVO_MIN_ANGLE			0
#define SERVO_MAX_ANGLE			180

#define ESC_MIN_WIDTH			10		//ms/10
#define ESC_MAX_WIDTH			20		//ms/10
#define ESC_MIN_ANGLE			0
#define ESC_MAX_ANGLE			180

#define SERVO_MAX			120
#define SERVO_MAX_PARTIAL	110
#define SERVO_ZERO			90
#define SERVO_MIN_PARTIAL	70
#define SERVO_MIN			60

#ifdef __cplusplus
extern "C"
{
#endif

void servo_init();
void servo_setPosition(int position);
void esc_setPosition(int position);

#ifdef __cplusplus
}
#endif
#endif /* SERVO_H_ */
