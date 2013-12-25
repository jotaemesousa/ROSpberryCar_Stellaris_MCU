/*
 * car.h
 *
 *  Created on: Dec 24, 2013
 *      Author: joao
 */

#ifndef CAR_H_
#define CAR_H_

#include "common_includes.h"
#include "remote_defines.h"
#include "Utilities/servo.h"
#include "Utilities/gpio_pwm_lights.h"

typedef struct ROSpberryCar
{
	int16_t velocity;
	int16_t batery_level;
	int32_t left_encoder;
	int32_t right_encoder;

}RC_Param;

typedef struct rc_cmds
{
	int Drive;
	int Steer;
	unsigned long int last_millis;
}RC_Cmds;

bool convert_values(RC_remote &in);
void updateCarParameters(void);

bool addNewLinearVelocity(int32_t v);
bool addNewAbgularVelocity(int32_t v);

void resetLeftEncoder(void);
void resetRightEncoder(void);

int32_t getLeftEncoder(void);
int32_t getRightEncoder(void);

void addLeftEncoder(int32_t deltaLeft);
void addRightEncoder(int32_t deltaRight);

void setLights(void);

#endif /* CAR_H_ */
