#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class Drivetrain
{
    public:
        Drivetrain();

        void reset_output();
        geometry_msgs::Twist get_output();
        void set_output(double linear_x, double linear_y, double linear_z, double angular_x, double angular_y, double angular_z);
        void set_output(double power, double turn);

    private:
        geometry_msgs::Twist output;
};

#endif