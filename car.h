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

bool convert_values(RC_remote &in, RC_Param &car_param, RC_Cmds &out);

#endif /* CAR_H_ */
