//User libs
#include "yeet_nav/pid_controller.h"

/**
 * @brief Construct a new pid controller::pid controller object
 * 
 */
PID_Controller::PID_Controller()
{
    this->last_time = ros::Time::now().toSec();
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
PID_Controller::PID_Controller(float cur_var)
{
    this->last_time = ros::Time::now().toSec();
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
void PID_Controller::reset(float cur_var)
{
    this->last_time = ros::Time::now().toSec();
    this->last_var = cur_var;
    this->integrator = 0;
    this->err = 0;
    this->prev_err = 0;
}

/**
 * @brief Gets the output from the PID
 * 
 * @param setpoint - The target value
 * @param cur_var - The current value
 * @return float - The output (p + i + d)
 */
float PID_Controller::getOutput(float setpoint, float cur_var)
{
    this->err = setpoint - cur_var;
    float p = KP * this->err;
    float dt = this->last_time - ros::Time::now().toSec();
    
    this->prev_err = setpoint - this->last_var;
    this->integrator += INT * (this->err + this->prev_err) * dt;
    float i = KI * this->integrator;

    float delta = (cur_var - this->last_var)/dt;
    float d = KD * delta;

    float output = coerce(p + i + d);

    this->last_var = cur_var;
    this->last_time = ros::Time::now().toSec();

    return output;
}

/**
 * @brief Clamps the output to max and min output if needed
 * 
 * @param pid_val - P + I + D
 * @return float - The coerced output
 */
float PID_Controller::coerce(float pid_val)
{
    pid_val = pid_val > MAX_OUTPUT ? MAX_OUTPUT : pid_val;
    pid_val = pid_val < MIN_OUTPUT ? MIN_OUTPUT : pid_val;

    return pid_val;
}