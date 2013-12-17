/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, ISR University of Coimbra.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the ISR University of Coimbra nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Gonçalo Cabrita and Bruno Antunes on 16/08/2012
 *********************************************************************/

#include "pid.h"

extern uint32_t millis();

pid::pid()
{
	reset();
	id_ = ' ';
	max_output_ = 255;
	min_output_ = -255;
	last_sensor_ = 0;
	alpha_ = 1.0;
	stop_flag_ = true;
	reference_ = 0;
	stop_threshould_ = 1;
	max_acc_error_ = 100;
	new_reference_ = true;
	debug_ = false;
	last_millis_ = millis();
	invert_ = false;
	accumulated_error_ = 0;
	last_error_ = 0;
}

void pid::setGains(float kp, float ki, float kd)
{
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
}


void pid::setSampleTime(float sample_time)
{
	sample_time_ = sample_time;
}

void pid::setNewReference(float ref, bool reset)
{
	reference_ = ref;

	if(reset)
	{
		stop_flag_ = false;
		new_reference_ = true;
	}
	if(last_sensor_ * ref <= 0)
	{
		invert_ = true;
	}
}

void pid::reset(void)
{
	if(!new_reference_)
	{
		last_error_ = 0.0;
	}

	accumulated_error_ = 0.0;
}

void pid::setMaxADC(int adc)
{
	max_adc_ = adc;

	if(max_adc_ > min_adc_) invert_ = false;
	else invert_ = true;
}

void pid::setMinADC(int adc)
{
	min_adc_ = adc;

	if(max_adc_ > min_adc_) invert_ = false;
	else invert_ = true;
}

int pid::run(float sensor)
{
	float temp = sensor * alpha_ + last_sensor_ * (1.0 - alpha_);

	sensor = temp;
	last_sensor_ = sensor;

	float kp, ki_dt, kd_dt;

	// Adaptative control
	// If the wheel is starting to move be more agressive
	kp = kp_;
	ki_dt = ki_ * sample_time_;
	kd_dt = kd_ / sample_time_;

	float error = -sensor + reference_;
	// calculate error
	if(invert_ && fabs(sensor) < 2)
	{
		invert_ = false;
	}

	// Calculate accumulated error
	accumulated_error_ += error;

	if(accumulated_error_ > max_acc_error_) accumulated_error_ = max_acc_error_;
	if(accumulated_error_ < -max_acc_error_) accumulated_error_ = -max_acc_error_;

	//if(accumulated_error_ > MAX_ACC_ERROR) accumulated_error_ = MAX_ACC_ERROR;
	//if(accumulated_error_ < -1*MAX_ACC_ERROR) accumulated_error_ = -1*MAX_ACC_ERROR;

	// Proportional term
	float p_term = kp * error;
	if(p_term > max_output_) p_term = max_output_;
	else if(p_term < min_output_) p_term = min_output_;

	// Integral term
	float i_term = ki_dt * accumulated_error_;
	if(i_term > max_output_) i_term = max_output_;
	else if(i_term < min_output_) i_term = min_output_;

	// Derivative term
	if(new_reference_)
	{
		last_error_ = error;
	}
	float d_term = kd_dt * (error - last_error_);
	if(d_term > max_output_) d_term = max_output_;
	else if(d_term < min_output_) d_term = min_output_;

	last_error_ = error;

	// Output
	int output = (int)(p_term + i_term + d_term);

	if(!invert_)
	{
		if(output > max_output_) output = max_output_;
		else if(output < min_output_) output = min_output_;
	}
	else
	{
		output = 0;
	}

	if(fabs(sensor) < 2 && fabs(reference_) < 1)
	{
		output = 0;
	}
//	if(stop_flag_)
//	{
//		output = 0;
//		accumulated_error_ = 0;
//		last_error_ = 0;
//		error = 0;
//	}
	new_reference_ = false;

	return output;
}

// EOF
