//User libs
#include "yeet_nav/pid_controller.h"

/**
 * @brief Construct a new pid controller::pid controller object
 * 
 */
PID_Controller::PID_Controller()
{
    this->last_var = 0;
    this->integrator = 0;
    this->err = 0;
    this->prev_err = 0;
}

/**
 * @brief Construct a new pid controller::pid controller object
 * 
 * @param cur_var - Current value
 */
PID_Controller::PID_Controller(double cur_var)
{
    this->last_var = cur_var;
    this->integrator = 0;
    this->err = 0;
    this->prev_err = 0;
}

/**
 * @brief Resets the PID if needed.
 * 
 * @param cur_var - Current value
 */
void PID_Controller::reset(double cur_var)
{
    this->last_time = ros::Time::now().toNSec() / 1000.0 / 1000.0 / 1000.0;
    this->last_var = cur_var;
    this->integrator = 0;
    this->err = 0;
    this->prev_err = 0;
}

void PID_Controller::init(double p, double i, double d, double max_out, double min_out)
{
    this->KP = p;
    this->KI = i;
    this->KD = d;
    this->MAX_OUTPUT = max_out;
    this->MIN_OUTPUT = min_out;
}

/**
 * @brief Gets the output from the PID
 * 
 * @param setpoint - The target value
 * @param cur_var - The current value
 * @return double - The output (p + i + d)
 */
double PID_Controller::getOutput(double setpoint, double process_var)
{
    double cur_time = ros::Time::now().toNSec() / 1000.0 / 1000.0 / 1000.0;
    this->err = setpoint - process_var;
    double p = KP * this->err;
    double dt = this->last_time - cur_time;
    
    this->prev_err = setpoint - this->last_var;
    this->integrator += (0.5) * (this->err + this->prev_err) * dt;
    double i = KI * this->integrator;

    double delta = (process_var - this->last_var)/dt;
    double d = KD * delta;

    double output = coerce(p + i + d);

    this->last_var = process_var;
    this->last_time = cur_time;

    return output;
}

/**
 * @brief Clamps the output to max and min output if needed
 * 
 * @param pid_val - P + I + D
 * @return double - The coerced output
 */
double PID_Controller::coerce(double pid_val)
{
    pid_val = pid_val > MAX_OUTPUT ? MAX_OUTPUT : pid_val;
    pid_val = pid_val < MIN_OUTPUT ? MIN_OUTPUT : pid_val;

    return pid_val;
}