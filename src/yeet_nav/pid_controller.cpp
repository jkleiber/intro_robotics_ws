//User libs
#include "yeet_nav/pid_controller.h"

/**
 * @brief Construct a new pid controller::pid controller object
 * 
 */
PID_Controller::PID_Controller()
{

}

//TODO: use PID instead
/**
 * @brief Turns to a target angle using a P controller (Proportional controller)
 * 
 * @param current_angle Current angle of the robot
 * @param target_angle Target angle
 * @return true The robots current angle is within tolerance of target angle
 * @return false The robot has not reached the target angle
 */
bool PID_Controller::turnToAngle(double current_angle, double target_angle, bool keep_going)
{
    //Declare local variables
    double error;
    double output;

    //Wrap the angles
    current_angle = this->angleWrap(current_angle);
    target_angle = this->angleWrap(target_angle);
    
    //Calculate error
    error = abs(current_angle - target_angle);

    if(!keep_going)
    {
        this->setPower(0);
    }

    //If the current angle is close enough to the target angle, then stop turning
    if(error < TURN_ERROR_TOLERANCE)
    {
        //Stop turning and relay the information that we made it to the target
        this->setTurn(0);
        return true;
    }
    //If the robot is erring to the left, turn right to get to the target
    else if(this->turnDirection(current_angle, target_angle))
    {
        output = this->clamp(KP * error, MAX_OUTPUT, MIN_OUTPUT);
        this->setTurn(output);
    }
    //If the robot is erring to the right, turn left to get to the target
    else
    {
        output = this->clamp(-(KP * error), MAX_OUTPUT, MIN_OUTPUT);
        this->setTurn(output);   
    }

    //Not in range of target yet
    return false;
}