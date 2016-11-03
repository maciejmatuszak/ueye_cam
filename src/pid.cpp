#include "ueye_cam/pid.hpp"

#include <ueye_cam/logging_macros.hpp>
#include <iostream>
#include <cmath>

PID::PID():_integral_queue(20)
{
    _pre_error = 0;
    _prevSampleTime = 0.0;
    _isRunning = false;
    _integral_queue_position = _integral_queue.begin();
}

//PID::PID( double dt, double min, double max, double Kp, double Ki, double Kd ) :

//    PID(),
//    _dt(dt),
//    _max(max),
//    _min(min),
//    _Kp(Kp),
//    _Kd(Kd),
//    _Ki(Ki)

//{
//}

void PID::updateParams(uint integral_history, double dt, double min, double max, double Kp_,  double Ki_, double Kd_)
{
    _dt = dt;
    _min = min;
    _max = max;
    Kp = Kp_;
    Kd = Kd_;
    Ki = Ki_;
    if(integral_history != _integral_queue.size())
    {
        //_integral_queue.resize(integral_history, 0.0);
        //_integral_queue_position = _integral_queue.begin();
    }
    ROS_DEBUG_STREAM("PID updated NO RESIZE: P:" << Kp << "; I: " << Ki << "; D: " << Kd);


}

double PID::calculateAsynch( double setpoint, double pv, double sampleTime)
{
    double dt;
    if(_isRunning == false)
    {
        dt = _dt;
        _isRunning = true;
    }
    else
    {
        dt = sampleTime - _prevSampleTime;
        _prevSampleTime = sampleTime;
    }

    return calculate(setpoint, pv, dt);
}

double PID::calculateSynch( double setpoint, double pv)
{
    return calculate(setpoint, pv, _dt);
}

double PID::calculate( double setpoint, double pv, double dt)
{

    // Calculate error
    error = setpoint - pv;
    value = pv;

    // Proportional term
    Pout = Kp * error;

    Iout =0.0;
    if(Ki != 0.0)
    {
        *_integral_queue_position = Kp * error;
        _integral_queue_position++;
        if(_integral_queue_position == _integral_queue.end())
        {
            _integral_queue_position = _integral_queue.begin();
        }

        for(uint i=0; i < _integral_queue.size(); i++)
        {
            Iout += _integral_queue[i];
            _integral_queue[i] *= 0.8;
        }

        Iout *= Ki;
    }

    // Derivative term
    double derivative = (error - _pre_error) / dt;
    _pre_error = error;
    Dout = Kd * derivative;

    // Calculate total output
    PIDout = Pout + Iout + Dout;
    ROS_DEBUG_STREAM("PID: value: " << pv << "; error: " << error << "; P Term: " << Pout << "; I term: " << Iout << "; D Term: " << Dout << "; Out: " << PIDout);

    // Restrict to max/min
    //if( output > _max )
    //    output = _max;
    //else if( output < _min )
    //    output = _min;


    return PIDout;
}

PID::~PID()
{
}
