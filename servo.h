/*
 * servo.h
 *
 *  Created on: Oct 15, 2012
 *      Author: bgouveia
 */

#ifndef SERVO_H_
#define SERVO_H_

// Servo Settings
#define SERVO_CENTER_ANGLE		93 //97
#define SERVO_LEFT_ANGLE		130 //133
#define SERVO_RIGHT_ANGLE		49


void servo_init();
void servo_setPosition(int position);



#endif /* SERVO_H_ */
