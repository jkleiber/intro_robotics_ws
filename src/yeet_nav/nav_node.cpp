//ROS libs and msgs
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

//User msgs and libs
#include <yeet_nav/pid_controller.h>
#include "yeet_msgs/node.h"

//Constants
#define RAD_TO_DEG (double)(180.0 / 3.14159)    //Conversion factor from radians to degrees
#define DISTANCE_TOL (double)(0.125)            //Tolerance for drive distance
#define ANGLE_TOL (double)(1.0)                 //Tolerance for angle distance
#define UP 0                                    //Up map angle
#define LEFT 90                                 //Left map angle
#define RIGHT -90                               //Right map angle
#define DOWN 180                                //Down map angle

//PID
PID_Controller turn;
PID_Controller drive;

//Global variables
yeet_msgs::Constants constants;
float current_angle;
int goal_row;
int goal_col;
int cur_row;
int cur_col;
float x;
float y

/**
 * @brief goalCallBack- Updates global variables for the PID Controller to use.
 * 
 * @param goal - The information about the robot's goal
 */
void goalCallBack(const yeet_msgs::node::ConstPtr& goal)
{
    goal_row = goal->row;
    goal_col = goal->col;
}

/**
 * @brief odomCallBack - Gets the current angle of the robot in degrees
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

    //Get the current row and column from x and y position
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;
    cur_col = (int) round(x / constants.SQUARE_SIZE);
    cur_row = (int) round(y / constants.SQUARE_SIZE);
}

/**
 * @brief angleWrap - Keep angles within the expected range
 * 
 * @param angle - Unwrapped angle
 * @return double - Angle between 0-360
 */
double angleWrap(double angle)
{
    return angle < 0 ? fmod(angle, 360) + 360 : fmod(angle, 360);
}

/**
 * @brief sweep - 
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

    //Set the loop rate of the nav function to 100 Hz
    ros::Rate loop_rate(100);

    //Create local messages
    yeet_msgs::move move;
    yeet_msgs::status status;

    //The callback and logic loop
    while(ros:ok())
    {
        ros:SpinOnce(); 

        //Get the difference in rows and columns
        int col_diff = cur_col - goal_col;
        int row_diff = cur_row - goal_row;
        int map_angle;
        //Down a square
        map_angle = (row_diff == 0 && col_diff == 1) ? DOWN : map_angle; 
        //Right a sqaure
        map_angle = (row_diff == 1 && col_diff == 0) ? RIGHT : map_angle;
        //Up a square
        map_angle = (row_diff == 0 && col_diff == -1) ? UP : map_angle;
        //Left a square
        map_angle = (row_diff == -1 && col_diff == 0) ? LEFT : map_angle;
        //Within tolerance, stop turning and start driving
        if(abs(sweep(map_angle)) <= ANGLE_TOL)
        {
            move.turn = 0;
            move.drive = (map_angle == DOWN || map_angle == UP) ? drive.getOutput(goal_row * constants.SQUARE_SIZE, x) : move.drive;
            move.drive = (map_angle == LEFT || map_angle == RIGHT) ? drive.getOutput(goal_col * constants.SQUARE_SIZE, y) : move.drive;
        }
        //Keep turning and do not drive
        else
        {
            move.turn = turn.getOutput(0, sweep(map_angle));
            move.drive = 0;
        }

        move_pub.publish(move);
        status_pub.publish(status);
    }
}