#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

//ROS and systems libs
#include <ros/ros.h>
#include <ros/console.h>


class PID_Controller
{
    public:
    PID_Controller();
    PID_Controller(double cur_var);
    void init(double p, double i, double d, double max_out, double min_out);
    void reset(double cur_var);
    double getOutput(double setpoint, double process_var);

    private:
    double coerce(double pid_val);
    double integrator;
    double last_time;
    double last_var;
    double err;
    double prev_err;

    double KP;
    double KI;
    double KD;
    double MAX_OUTPUT;
    double MIN_OUTPUT;

};

#endif