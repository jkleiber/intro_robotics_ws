#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

//ROS and systems libs
#include <ros/ros.h>
#include <ros/console.h>

//Constants
#define KP (float)(0.18)
#define KI (float)(0.001)
#define KD (float)(0.001)
#define INT (float)(0.5)
#define MAX_OUTPUT 0.75
#define MIN_OUTPUT -0.75

class PID_Controller
{
    public:
    PID_Controller();
    PID_Controller(float cur_var);
    void reset(float cur_var);
    float getOutput(float setpoint, float endpoint);

    private:
    float coerce(float pid_val);
    float integrator;
    float last_time;
    float last_var;
    float err;
    float prev_err;

};

#endif