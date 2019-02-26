#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

//ROS and system libs
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>

//Motion error tolerances
#define TURN_ERROR_TOLERANCE (double)(1.0)

//Output constants
#define MAX_OUTPUT 0.75
#define MIN_OUTPUT -0.75

//PID constants
#define KP (double)(0.18)

class Drivetrain
{
    public:
        Drivetrain();

        void resetOutput();

        geometry_msgs::Twist getOutput();
        
        void setOutput(geometry_msgs::Twist output_data);
        void setOutput(double power, double turn);
        void setTurn(double turn);
        void setPower(double power);
        
        double angleWrap(double angle);
        bool turnDirection(double start_angle, double end_angle);
        bool turnToAngle(double currentAngle, double target_angle);

    private:
        double clamp(double x, double max_out, double min_out);
        void setOutput(double linear_x, double linear_y, double linear_z, double angular_x, double angular_y, double angular_z);

        geometry_msgs::Twist output;
};

#endif