#include <ros/ros.h>

//Scanner libs and msgs
#include <sensor_msgs/LaserScan.h>

//User libs and msgs
#include <reactive_robot/obstacle.h>

//Publisher
ros::Publisher obstacle_pub;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& obstacle_event)
{
    
}
/**
 * Runs the loop needed to handle obstacle detection 
 */
int main(int argc, char **argv)
{
    //Start the node
    ros::init(argc, argv, "obstacle_avoid_node");

    //Set up the node handle for auto driving
    ros::NodeHandle obstacle_avoid_node;

    //Subscribe to the scanner
    ros::Subscriber obstacle_sub = obstacle_node.subscribe(obstacle_node.resolveName("/scan"), 10, &scanCallback);

    //Publish state to the obstacle topic
    obstacle_pub = obstacle_node.advertise<reactive_robot::obstacle>(obstacle_node.resolveName("/reactive_robot/obstacle"), 10);

    //Handle the callbacks
    ros::spin();
}
