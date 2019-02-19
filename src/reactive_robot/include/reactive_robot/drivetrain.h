#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

//ROS and system libs
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//User includes
#include "reactive_robot/constants.h"

//Motion error tolerances
#define TURN_ERROR_TOLERANCE (double)(1.0)

//PID constants
#define Kp (double) 

class Drivetrain
{
    public:
        Drivetrain();

        void resetOutput();

        geometry_msgs::Twist getOutput();
        
        void setOutput(geometry_msgs::Twist output_data);
        void setOutput(double linear_x, double linear_y, double linear_z, double angular_x, double angular_y, double angular_z);
        void setOutput(double power, double turn);
        void setTurn(double turn);
        
        double angleWrap(double angle);
        bool turnDirection(double start_angle, double end_angle);
        bool turnToAngle(double currentAngle, double target_angle);

    private:
        geometry_msgs::Twist output;
};

#endif