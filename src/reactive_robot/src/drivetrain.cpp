//User libs and msgs
#include "reactive_robot/drivetrain.h"

/**
 * @brief Construct a new Drivetrain:: Drivetrain object
 */
Drivetrain::Drivetrain()
{
    //Start the drivetrain at a stopped state
    this->resetOutput();
}


/**
 * @brief Stops the robot
 */
void Drivetrain::resetOutput()
{
    this->setOutput(0, 0, 0, 0, 0, 0);
}


/**
 * @brief Get the current drivetrain output
 * 
 * @return geometry_msgs::Twist 
 */
geometry_msgs::Twist Drivetrain::getOutput()
{
    return this->output;
}


/**
 * @brief  Set the drivetrain output from a Twist message
 * 
 * @param output_data Twist message containing output information
 */
void Drivetrain::setOutput(geometry_msgs::Twist output_data)
{
    this->output = output_data;
}


/**
 * @brief Set the drivetrain speed and turn velocity
 * 
 * @param power Linear speed
 * @param turn Angular speed
 */
void Drivetrain::setOutput(double power, double turn)
{
    this->setTurn(turn);
    this->setPower(power);
}


/**
 * @brief turns the robot in place at a certain power
 * 
 * @param turn The angular velocity
 */
void Drivetrain::setTurn(double turn)
{
    this->output.angular.z = turn;
}


/**
 * @brief sets the throttle for the linear motion of the robot
 * 
 * @param power Robot speed
 */
void Drivetrain::setPower(double power)
{
    this->output.linear.x = power;
}


/**
 * @brief Keep angles within the expected range
 * 
 * @param angle Unwrapped angle
 * @return double Angle between 0-360
 */
double Drivetrain::angleWrap(double angle)
{
    return angle < 0 ? fmod(angle, 360) + 360 : fmod(angle, 360);
}


/**
 * @brief Determine which direction the robot should turn (right/left)
 * 
 * @param start_angle Beginning angle of the robot
 * @param end_angle End angle of the robot
 * @return true Robot should turn right
 * @return false Robot should turn left
 */
bool Drivetrain::turnDirection(double start_angle, double end_angle)
{
    //Find angles wrapped from [0, 360)
    double start_wrapped = angleWrap(start_angle);
    double end_wrapped = angleWrap(end_angle);

    //Find the wrapped sweep between the start and end angle
    double delta = angleWrap(end_wrapped - start_wrapped);

    //Turn left if delta is less than 180, right otherwise
    return delta <= 180 ? true : false;
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
bool Drivetrain::turnToAngle(double current_angle, double target_angle, bool keep_going)
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


/**
 * @brief Clamps the output to a max speed
 * 
 * @param x Input variable to clamp
 * @param max_out Max speed clamp
 * @param min_out Min speed clamp
 * @return double Clamped input variable
 */
double Drivetrain::clamp(double x, double max_out, double min_out)
{
    //If input variable is greater than max or less than min, clamp and return
    x = x > max_out ? max_out : x;
    x = x < min_out ? min_out : x;

    return x;
}


/**
 * @brief Updates the angular and linear velocity of the robot
 * 
 * @param linear_x Linear x velocity
 * @param linear_y Linear y velocity
 * @param linear_z Linear z velocity
 * @param angular_x Angular x velocity
 * @param angular_y Angular y velocity
 * @param angular_z Angular z velocity
 */
void Drivetrain::setOutput(double linear_x, double linear_y, double linear_z, 
    double angular_x, double angular_y, double angular_z)
{
    //Update all velocities
    this->output.linear.x = linear_x;
    this->output.linear.y = linear_y;
    this->output.linear.z = linear_z;
    this->output.angular.x = angular_x;
    this->output.angular.y = angular_y;
    this->output.angular.z = angular_z;
}