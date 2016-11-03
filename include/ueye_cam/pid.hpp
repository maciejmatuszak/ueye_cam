#ifndef PID_HPP
#define PID_HPP
#include <boost/lockfree/queue.hpp>


class PID
{
    public:
        PID();
        //PID( double dt, double min, double max, double Kp, double Ki, double Kd );
        ~PID();
        double calculate( double setpoint, double pv, double dt);
        double calculateSynch( double setpoint, double pv);
        double calculateAsynch( double setpoint, double pv, double sampleTime);
        void updateParams(uint integral_history, double dt, double min, double max, double Kp,  double Ki, double Kd);
        double Kp;
        double Kd;
        double Ki;
        double Pout;
        double Iout;
        double Dout;
        double PIDout;
        double value;
        double error;


    private:
        bool _isRunning;
        double _dt;
        double _prevSampleTime;
        double _max;
        double _min;
        double _pre_error;
        uint _integralSamples;
        std::vector<double> _integral_queue;
        std::vector<double>::iterator _integral_queue_position;
};

#endif // PID_HPP
