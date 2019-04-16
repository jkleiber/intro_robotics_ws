//ROS libs and msgs
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

//User msgs and libs
#include <yeet_nav/pid_controller.h>
#include <yeet_msgs/node.h>

//Constants
#define RAD_TO_DEG (double)(180.0 / 3.14159)    //Conversion factor from radians to degrees

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
 * @brief - Updates global variables for the PID Controller to use.
 * 
 * @param goal - The information about the robot's goal
 */
void goalCallBack(const yeet_msgs::node::ConstPtr& goal)
{
    goal_x = goal->real_x;
    goal_y = goal->real_y;
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
    cur_x = current->real_x;
    cur_y = current->real_y;
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
    current_angle = drivetrain.angleWrap(tf::getYaw(pose.getRotation()) * RAD_TO_DEG);
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
        nav_node.resolveName("/yeet_msgs/node"), 10, &goalCallBack);
    ros::Subscriber current_sub = nav_node.subscribe(
        nav_node.resolveName("/yeet_msgs/node"), 10, &currentCallBack);
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