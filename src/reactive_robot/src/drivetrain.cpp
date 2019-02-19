#include "reactive_robot/drivetrain.h"

/**
 * 
 */
Drivetrain::Drivetrain()
{
    this->resetOutput();
}


/**
 * 
 */
void Drivetrain::resetOutput()
{
    this->output.angular.x = 0;
    this->output.angular.y = 0;
    this->output.angular.z = 0;
    this->output.linear.x = 0;
    this->output.linear.y = 0;
    this->output.linear.z = 0;
}


/**
 * 
 */
geometry_msgs::Twist Drivetrain::getOutput()
{
    return this->output;
}


/**
 * 
 */
void Drivetrain::setOutput(double linear_x, double linear_y, double linear_z, double angular_x, double angular_y, double angular_z)
{
    this->output.linear.x = linear_x;
    this->output.linear.y = linear_y;
    this->output.linear.z = linear_z;
    this->output.angular.x = angular_x;
    this->output.angular.y = angular_y;
    this->output.angular.z = angular_z;
}


/**
 * 
 */
void Drivetrain::setOutput(double power, double turn)
{
    this->setOutput(power, 0, 0, 0, 0, turn);
}

/**
 * 
 */
double Drivetrain::angleWrap(double angle)
{
    if(angle < 0)
    {
        return fmod(angle, 360) + 360;
    }
    else
    {
        return fmod(angle, 360);
    }
}

/**
 * 
 * 
 * @return  True if the robot should turn right
 *          False if the robot should turn left
 */
bool Drivetrain::turnDirection(double start_angle, double end_angle)
{
    //TODO: Decide if we use angleWrap() here or outsude. 
    double start_wrapped = angleWrap(start_angle);
    double end_wrapped= angleWrap(end_angle);

    //Determine which direciton will be faster to turn
    if((start_wrapped + 360 - end_wrapped) < 180)
    {
        //Turn right
        return false;
    }
    else
    {
        //Turn left
        return true;
    }
}