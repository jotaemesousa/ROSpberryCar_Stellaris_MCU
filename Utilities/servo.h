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
#define SERVO_CENTER_ANGLE		96 //97
#define SERVO_LEFT_ANGLE		132//133
#define SERVO_RIGHT_ANGLE		55

#ifdef __cplusplus
extern "C"
{
#endif

void servo_init();
void servo_setPosition(int32_t position);
int32_t servo_valueMappedToAngle(int8_t value);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H_ */
