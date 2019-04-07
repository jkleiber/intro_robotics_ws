#include <ros/ros.h>

//TODO: main purpose of this file is to take gmapping data and fit it to our map
//TODO: send map updates rather than full maps. For example, if an obstacle appears say "on F6 there is an unexpected obstacle"

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "mapping_node");

    //Set up node controller
    ros::NodeHandle map_node;

    //Subscribe to SLAM updates from gmapping
}