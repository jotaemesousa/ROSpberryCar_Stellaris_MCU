/*
 * car.cpp
 *
 *  Created on: Dec 24, 2013
 *      Author: joao
 */
#include "car.h"

static RC_Param car_param;
static RC_Cmds out;
static uint32_t last_millis_pid = 0;
pid velocity_pid = pid();
INA226 power_meter;

void initCarPID(void)
{
	velocity_pid.setGains(5.0,2.5,0.0);
	velocity_pid.setSampleTime(0.050);
	velocity_pid.setMaxAccumulatedError(40);
	velocity_pid.setFilter(0.20);
	velocity_pid.setMaxOutput(50);
	velocity_pid.setMinOutput(-50);
	velocity_pid.initSensor(0);
	velocity_pid.setNewReference(0,1);
}

void initINA226(void)
{
	power_meter = INA226(INA226_I2C_ADDR);
	power_meter.set_sample_average(4);
	power_meter.set_calibration_value(445);
	power_meter.set_bus_voltage_limit(7.0);
	power_meter.set_mask_enable_register(BUS_UNDER_LIMIT);
}

bool convert_values(RC_remote &in)
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

void updateCarParameters(void)
{
	if(millis() - last_millis_pid > 50)
	{
		last_millis_pid = millis();

		int32_t le = 0, re=0, out = 0;
		encoder_read_reset(&le, &re);
		out = velocity_pid.run((le + re)/2);
		drive_pwm(out,1);
	}
}

bool addNewLinearVelocity(int32_t v)
{
	velocity_pid.setNewReference((float)v,0);
	return false;
}

bool addNewAngularVelocity(int32_t v)
{
	servo_setPosition(servo_valueMappedToAngle(v));
	return false;
}

void resetLeftEncoder(void)
{
	car_param.left_encoder = 0;
}

void resetRightEncoder(void)
{
	car_param.right_encoder = 0;
}

int32_t getLeftEncoder(void)
{
	return car_param.left_encoder;
}

int32_t getRightEncoder(void)
{
	return car_param.right_encoder;
}

void addLeftEncoder(int32_t deltaLeft)
{
	car_param.right_encoder += deltaLeft;
}

void addRightEncoder(int32_t deltaRight)
{
	car_param.right_encoder += deltaRight;
}

void setLights(void)
{

}


