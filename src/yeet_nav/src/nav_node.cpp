//ROS libs and msgs
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

//User msgs and libs
#include "yeet_nav/pid_controller.h"
#include "yeet_msgs/node.h"
#include "yeet_msgs/move.h"
#include "yeet_msgs/nav_status.h"

//Constants
#define RAD_TO_DEG (double)(180.0 / 3.14159)    //Conversion factor from radians to degrees
#define DISTANCE_TOL (double)(0.125)            //Tolerance for drive distance
#define ANGLE_TOL (double)(1.0)                 //Tolerance for angle distance

//PID
PID_Controller turn;
PID_Controller drive;

//Global variables
float current_angle;
float goal_x;
float goal_y;
float cur_x;
float cur_y;
int goal_row;
int goal_col;
int cur_row;
int cur_col;

/**
 * @brief - Keep angles within the expected range
 * 
 * @param angle - Unwrapped angle
 * @return double - Angle between 0-360
 */
double angleWrap(double angle)
{
    return angle < 0 ? fmod(angle, 360) + 360 : fmod(angle, 360);
}

/**
 * @brief - Updates global variables for the PID Controller to use.
 * 
 * @param goal - The information about the robot's goal
 */
void goalCallBack(const yeet_msgs::node::ConstPtr& goal)
{
    goal_row = goal->row;
    goal_col = goal->col;
}

/**
 * @brief - Updates global variables for the PID Controller to use.
 * 
 * @param current - The information about the robot's current positions
 */
void currentCallBack(const yeet_msgs::node::ConstPtr& current)
{
    cur_row = current->row;
    cur_col = current->col;
}

/**
 * @brief - Gets the current angle of the robot in degrees
 * 
 * @param odom - The odometry message containing robot angle position
 */
void odomCallBack(const nav_msgs::Odometry::ConstPtr& odom)
{
    //Get the robot orientation
    tf::Pose pose;
    tf::poseMsgToTF(odom->pose.pose, pose);

    //Get the current angle in degrees
    current_angle = angleWrap(tf::getYaw(pose.getRotation()) * RAD_TO_DEG);
}

/**
 * @brief - 
 * 
 * @param target_angle - The desired turn angle
 * @return double - Returns the error in angle. Negative if the turn
 * should be to the right, positive if left.
 */
double sweep(float target_angle)
{
    double sweep = target_angle - current_angle;
    sweep = (sweep >  180) ? sweep - 360 : sweep;
    sweep = (sweep < -180) ? sweep + 360 : sweep;
    return sweep;
}

/**
 * @brief - Main method
 * 
 * @param argc - Number of args
 * @param argv - Args into the executable
 * @return int - Exit code
 */
int main(int argc, char **argv)
{
    //Start the node
    ros::init(argc, argv, "nav_node");

    //Set up the node to handle motion
    ros::NodeHandle nav_node;

    //Subscribe to topics
    ros::Subscriber goal_sub = nav_node.subscribe(
        nav_node.resolveName("/yeet_planning/target_node"), 10, &goalCallBack);
    ros::Subscriber odom_sub = nav_node.subscribe(
        nav_node.resolveName("/odom"), 10, &odomCallBack);

    //Publishers
    ros::Publisher move_pub = nav_node.advertise<yeet_msgs::move>(
        nav_node.resolveName("/yeet_nav/navigation"), 10);
    ros::Publisher status_pub = nav_node.advertise<yeet_msgs::nav_status>(
        nav_node.resolveName("/yeet_nav/status"), 10);

    //Set the loop rate of the nav function to 200 Hz
    ros::Rate loop_rate(200);

    while(ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}