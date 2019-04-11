//ROS libs and msgs
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

//User msgs and libs
#include <yeet_nav/pid_controller.h>
#include <yeet_msgs/node.h>

//Constants
#define RAD_TO_DEG (double)(180.0 / 3.14159)    //Conversion factor from radians to degrees

PID_Controller turn;
PID_Controller drive;

double current_angle;

void goalCallBack(const yeet_planning::next_node::ConstPtr& goal)
{

}

void currentCallBack(const yeet_planning::current_node::ConstPtr& current)
{

}

void odomCallBack(const nav_msgs::Odometry::ConstPtr& odom)
{
    //Get the robot orientation
    tf::Pose pose;
    tf::poseMsgToTF(odometer->pose.pose, pose);

    //Get the current angle in degrees
    current_angle = drivetrain.angleWrap(tf::getYaw(pose.getRotation()) * RAD_TO_DEG);
}

/**
 * @brief Main method
 * 
 * @param argc Number of args
 * @param argv Args into the executable
 * @return int Exit code
 */
int main(int argc, char **argv)
{
    //Start the node
    ros::init(argc, argv, "nav_node");

    //Set up the node to handle motion
    ros::NodeHandle nav_node;

    //Subscribe to topics
    ros::Subscriber goal_sub = nav_node.subscribe(
        nav_node.resolveName("/yeet_planning/next_node"), 10, &goalCallBack);
    ros::Subscriber current_sub = nav_node.subscribe(
        nav_node.resolveName("/yeet_planning/current_node"), 10, &currentCallBack);
    ros::Subscriber odom_sub = nav_node.subscribe(
        nav_node.resolveName("/odom"), 10, &odomCallBack);

    //Publish to the turtlebot's cmd_vel_mux topic
    ros::Publisher move_pub = nav_node.advertise<yeet_msgs::move>(
        nav_node.resolveName("/yeet_nav/navigation"), 10);

    //Set the loop rate of the nav function to 100 Hz
    ros::Rate loop_rate(100);

    while(ross:ok())
    {
        ros:SpinOnce();
    }
}