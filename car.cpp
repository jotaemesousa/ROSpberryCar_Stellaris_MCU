/*
 * car.cpp
 *
 *  Created on: Dec 24, 2013
 *      Author: joao
 */
#include "car.h"


bool convert_values(RC_remote &in, RC_Param &car_param, RC_Cmds &out)
{
	float steer_factor = 1;
	static bool steer_mode = 0;

	if(car_param.velocity < VEL_STEER_MIN && steer_mode == 1)
	{
		steer_factor = 1;
		steer_mode = 0;
	}
	else if(car_param.velocity > VEL_STEER_MAX && steer_mode == 0)
	{
		if(in.steer > 0)
		{
			steer_factor = (pow(in.steer / 127.0, 2) + in.steer/127.0) / 2.0;
		}
		else
		{
			steer_factor = (pow(in.steer / 127.0, 2) - in.steer/127.0) / 2.0;
		}
		steer_mode = 0;
	}

	int d,s;

	d = in.linear;
	s = in.steer * steer_factor;

	int max_vel_fwd = 0, max_vel_rev = 0;
	if((in.buttons & L1_BUTTON) == L1_BUTTON)
	{
		max_vel_fwd = DRV_FRONT_N2O;
		max_vel_rev = DRV_REAR_N2O;
	}
	else
	{
		max_vel_fwd = DRV_FRONT;
		max_vel_rev = DRV_REAR;
	}

	if(d == 0){
		out.Drive = DRV_ZERO;
	}
	else if(d <= -127)
	{
		out.Drive = max_vel_rev;
	}
	else if(d >= 127)
	{
		out.Drive = max_vel_fwd;
	}
	else if(d < 0 && d > -127)
	{
		out.Drive = (-d * (max_vel_rev - DRV_ZERO))/(127) + DRV_ZERO;
	}
	else if(d > 0 && d < 127)
	{
		out.Drive = ((d * (max_vel_fwd - DRV_ZERO))/127) + DRV_ZERO;
	}

	if(s == 0)
	{
		out.Steer = SERVO_CENTER_ANGLE;
	}
	else if(s <= -127)
	{
		out.Steer = SERVO_LEFT_ANGLE;
	}
	else if(s >= 127)
	{
		out.Steer = SERVO_RIGHT_ANGLE;
	}
	else if(s < 0 && s > -127)
	{
		out.Steer = (-s * (SERVO_LEFT_ANGLE - SERVO_CENTER_ANGLE))/(127) + SERVO_CENTER_ANGLE;
	}
	else if(s > 0 && s < 127)
	{
		out.Steer = ((s * (SERVO_RIGHT_ANGLE - SERVO_CENTER_ANGLE))/127) + SERVO_CENTER_ANGLE;
	}

	return true;
}

