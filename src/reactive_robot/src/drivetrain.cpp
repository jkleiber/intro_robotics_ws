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
    this->setOutput(0, 0, 0, 0, 0, 0);
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
void Drivetrain::setOutput(geometry_msgs::Twist output_data)
{
    this->output = output_data;
}



/**
 * 
 */
void Drivetrain::setOutput(double power, double turn)
{
    this->setTurn(turn);
    this->setPower(power);
}


/**
 * setTurn - turns the robot in place at a certain power
 */ 
void Drivetrain::setTurn(double turn)
{
    this->output.angular.z = turn;
}


/**
 * setPower - sets the throttle for the linear motion of the robot
 */
void Drivetrain::setPower(double power)
{
    this->output.linear.x = power;
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
    //Find angles wrapped from [0, 360)
    double start_wrapped = angleWrap(start_angle);
    double end_wrapped = angleWrap(end_angle);

    //Find the wrapped sweep between the start and end angle
    double delta = angleWrap(end_wrapped - start_wrapped);

    //Determine which direction will be faster to turn
    if(delta > 180)
    {
        //Turn left
        return false;
    }

    //Turn right
    return true;
}




//TODO: use PID instead
/**
 * 
 */
bool Drivetrain::turnToAngle(double current_angle, double target_angle)
{
    //Declare local variables
    double error;
    double output;

    //Wrap the angles
    current_angle = this->angleWrap(current_angle);
    target_angle = this->angleWrap(target_angle);

    //Debug
    //ROS_INFO("Current Angle: %f, Target Angle: %f -> %d", current_angle, target_angle, this->turnDirection(current_angle, target_angle));
    
    
    //Calculate error
    error = abs(current_angle - target_angle);

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
        this->setPower(0);
        this->setTurn(output);
    }
    //If the robot is erring to the right, turn left to get to the target
    else
    {
        output = this->clamp(-(KP * error), MAX_OUTPUT, MIN_OUTPUT);
        this->setPower(0);
        this->setTurn(output);   
    }

    //Not in range of target yet
    return false;
}




/**
 * clamp - make sure a variable x does not exceed a maximum or fall below a minimum
 */
double Drivetrain::clamp(double x, double max_out, double min_out)
{
    x = x > max_out ? max_out : x;
    x = x < min_out ? min_out : x;

    return x;
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