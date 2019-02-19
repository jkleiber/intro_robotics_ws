#include "reactive_robot/drivetrain.h"

/**
 * 
 */
Drivetrain::Drivetrain()
{
    this->reset_output();
}


/**
 * 
 */
void Drivetrain::reset_output()
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
geometry_msgs::Twist Drivetrain::get_output()
{
    return this->output;
}


/**
 * 
 */
void Drivetrain::set_output(double linear_x, double linear_y, double linear_z, double angular_x, double angular_y, double angular_z)
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
void Drivetrain::set_output(double power, double turn)
{
    this->set_output(power, 0, 0, 0, 0, turn);
}