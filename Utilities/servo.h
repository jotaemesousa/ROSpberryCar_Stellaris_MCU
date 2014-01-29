/*
 * servo.h
 *
 *  Created on: Oct 15, 2012
 *      Author: bgouveia
 */

#ifndef SERVO_H_
#define SERVO_H_

// Servo Settings
#define SERVO_CENTER_ANGLE		97 //97
#define SERVO_LEFT_ANGLE		132//133
#define SERVO_RIGHT_ANGLE		55

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


void servo_init();
void servo_setPosition(int position);
void esc_setPosition(int position);


#endif /* SERVO_H_ */
