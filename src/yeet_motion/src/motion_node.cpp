//ROS libs and msgs
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//User libs and msgs
#include "yeet_msgs/move.h"

//Movement message
geometry_msgs::Twist twist;

//ROS Publishers
ros::Publisher teleop_pub;

void movementCallback(const yeet_msgs::move::ConstPtr& move_event)
{
    twist.linear.x = move_event->drive;
    twist.angular.z = move_event->turn;
    teleop_pub.publish(twist);
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
    ros::init(argc, argv, "motion_node");

    //Set up the node to handle motion
    ros::NodeHandle motion_node;

    //Subscribe to topics
    ros::Subscriber movement_sub = motion_node.subscribe(
        motion_node.resolveName("/yeet_mergency/human_control"), 10, &movementCallback);

    //Publish to the turtlebot's cmd_vel_mux topic
    teleop_pub = motion_node.advertise<geometry_msgs::Twist>(motion_node.resolveName("/cmd_vel_mux/input/teleop"), 10);

    ros::spin();

    return 0;
}