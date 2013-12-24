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

bool convert_values(RC_remote &in, RC_Param &car_param, RC_Cmds &out);

#endif /* CAR_H_ */
