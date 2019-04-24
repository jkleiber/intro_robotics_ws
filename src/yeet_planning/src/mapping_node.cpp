#include <ros/ros.h>

//TODO: main purpose of this file is to take gmapping data and fit it to our map
//TODO: send map updates rather than full maps. For example, if an obstacle appears say "on F6 there is an unexpected obstacle"

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "mapping_node");

    //Set up node controller
    ros::NodeHandle slam_node;

    //Subscribe to laser scanner and bump sensors
    //TODO: take namespaces as params
    ros::Subscriber laser_sub = slam_node.subscribe(slam_node.resolveName("/scan"), MAX_BUFFER, &scanCallback);
    ros::Subscriber bump_sub = slam_node.subscribe(slam_node.resolveName("/mobile_base/events/bumper"), MAX_BUFFER, &bumperCallback);
    
    //Subscribe to SLAM updates from gmapping?
    
}