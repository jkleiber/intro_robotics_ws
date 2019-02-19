#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Drivetrain
{
    public:
        Drivetrain();

        void resetOutput();
        geometry_msgs::Twist getOutput();
        void setOutput(double linear_x, double linear_y, double linear_z, double angular_x, double angular_y, double angular_z);
        void setOutput(double power, double turn);
        double angleWrap(double angle);
        bool Drivetrain::turnDirection(double start_angle, double end_angle);



    private:
        geometry_msgs::Twist output;
};

#endif