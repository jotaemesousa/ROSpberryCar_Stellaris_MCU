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
* Author: Gonçalo Cabrita and Bruno Antunes  e JMS :p on 16/08/2012
*********************************************************************/

#ifndef PID_h
#define PID_h

#include "math.h"
#include "stdint.h"

#define STARTING_PID                                  0
#define RUNNING_PID                                   1
#define INTEGRAL_ERROR_RESET_TIME        5000

#define AUTOTUNE_MAX_VALUES 60

class pid
{
public:
    pid();

    void setGains(float kp, float ki, float kd);

    float getKp() {return kp_;}
    float getKi() {return ki_;}
    float getKd() {return kd_;}

    void setSampleTime(float sample_time);

    void setMaxOutput(int value) {max_output_ = value;}
    void setMinOutput(int value) {min_output_ = value;}
    void setStopThreshould(float value) {stop_threshould_ = fabs(value);}

    void setNewReference(float ref, bool reset = 1);
    float getReference(void){return reference_;}
    float getLastSensor(void){return last_sensor_;}

    void setFilter(float percent){alpha_ = percent;}

    int run(float input);

    void reset(void);

    void setMaxAccumulatedError(float max){ max_acc_error_ = max;}
    float getMaxAccumulatedError(void){ return max_acc_error_;}

    void setPIDid(char i){ id_ = i;}

    void setDebugOn(void){debug_ = true;}
    void setDebugOff(void){debug_ = false;}

    void setMaxADC(int adc);
    void setMinADC(int adc);

    void initSensor(float last_sensor){last_sensor_ = last_sensor;}

private:

    float kp_;
    float ki_;
    float kd_;

    float sample_time_;

    float last_error_;
    float accumulated_error_, max_acc_error_;

    float state_;

    int max_output_, min_output_;
    float alpha_;

    float last_sensor_;

    float reference_;
    bool stop_flag_, new_reference_;
    float stop_threshould_;
    unsigned long int last_millis_;

    char id_;
    bool debug_;

    int max_adc_, min_adc_;
    bool invert_;
};

#endif

// EOF
