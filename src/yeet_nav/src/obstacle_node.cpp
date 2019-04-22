//ROS msgs and libs
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


//User libs and msgs
#include <yeet_msgs/obstacle.h>


//Other libs
#include <cmath>
#include <math.h>


//Constants
#define TOTAL_SAMPLES 640                                       //Laser scan samples
#define N_CENTER_SAMPLES 75                                     //Width of center view region
#define N_SIDE_SAMPLES ((TOTAL_SAMPLES - N_CENTER_SAMPLES) / 2) //Allocate the number of samples for the side views
#define LEFT_SAMPLES_IDX TOTAL_SAMPLES - N_SIDE_SAMPLES         //Where the left samples start
#define RIGHT_SAMPLES_IDX N_SIDE_SAMPLES                        //Where the right samples end
#define DETECT_CONST (double)(.3)

yeet_msgs::Constants constants;

//Publisher
ros::Publisher obstacle_pub;

/**
 * @brief - Detects obstacles in front of the robot
 * 
 * @param obstacle_event: the message sent from the LaserScan sensor messages containing the information of the scan
 */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& obstacle_event)
{
    //Declare the obstacle message as a local variable
    reactive_robot::obstacle obstacle_msg;
    //Set the obstacle detection to false
    obstacle_msg.obstacle = false;

    //Loop through all the samples and update bool if any index has an object
    for(int i = RIGHT_SAMPLES_IDX; i < LEFT_SAMPLES_IDX; ++i)
    {
         //If an obstacle is detected in the next cell
        if(!std::isnan(obstacle_event->ranges[i]) && obstacle_event->ranges[i] <= constants.SQUARE_SIZE + DETECT_CONST)
        {
            //Update obstacle boolean to true
            obstacle_msgs.obstacle = true;
            break;
        }        
    }
    //Publish
    obstacle_pub.publish(obstacle_msg);
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
    ros::init(argc, argv, "obstacle_avoid_node");

    //Set up the node handle for auto driving
    ros::NodeHandle obstacle_avoid_node;

    //Subscribe to the scanner
    ros::Subscriber obstacle_sub = obstacle_avoid_node.subscribe(
        obstacle_avoid_node.resolveName("/scan"), 10, &scanCallback);

    //Publish state to the obstacle topic
    obstacle_pub = obstacle_avoid_node.advertise<reactive_robot::obstacle>(
        obstacle_avoid_node.resolveName("/yeet_msgs/obstacle"), 10);

    //Handle the callbacks
    ros::spin();
}