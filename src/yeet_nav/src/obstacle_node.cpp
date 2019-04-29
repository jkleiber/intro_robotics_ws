//ROS msgs and libs
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


//User libs and msgs
#include "yeet_msgs/obstacle.h"
#include "yeet_msgs/move.h"
#include "yeet_msgs/Constants.h"

//Other libs
#include <cmath>
#include <math.h>


//Constants
#define TOTAL_SAMPLES 640                                       //Laser scan samples
#define N_CENTER_SAMPLES 75                                     //Width of center view region
#define N_SIDE_SAMPLES ((TOTAL_SAMPLES - N_CENTER_SAMPLES) / 2) //Allocate the number of samples for the side views
#define LEFT_SAMPLES_IDX TOTAL_SAMPLES - N_SIDE_SAMPLES         //Where the left samples start
#define RIGHT_SAMPLES_IDX N_SIDE_SAMPLES                        //Where the right samples end
#define DETECT_CONST (double)(0.0)

yeet_msgs::Constants constants;

//Publishers
ros::Publisher move_pub;
ros::Publisher obstacle_pub;
ros::Publisher replan_pub;

//Track state of obstacles
yeet_msgs::obstacle obstacle_msg;

/**
 * @brief - Detects obstacles in front of the robot
 * 
 * @param obstacle_event: the message sent from the LaserScan sensor messages containing the information of the scan
 */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& obstacle_event)
{
    //Loop through all the samples and update bool if any index has an object
    for(int i = RIGHT_SAMPLES_IDX; i < LEFT_SAMPLES_IDX; ++i)
    {
         //If an obstacle is detected in the next cell
        if(!std::isnan(obstacle_event->ranges[i]) && obstacle_event->ranges[i] <= constants.SQUARE_SIZE + DETECT_CONST)
        {
            //Update obstacle boolean to true
            obstacle_msg.obstacle = true;
            break;
        }
        else
        {
            //Set the obstacle detection to false
            obstacle_msg.obstacle = false;
        }
        
    }

    //TODO: is this publish needed?
    //Publish
    obstacle_pub.publish(obstacle_msg);
}


void navCallback(const yeet_msgs::move::ConstPtr& move_msg)
{
    yeet_msgs::move cmd;

    //Initialize the command to zeros
    cmd.drive = 0;
    cmd.turn = 0;

    //If there is an obstacle, inhibit driving forward
    if(obstacle_msg.obstacle)
    {
        printf("Obstacle detected! Inhibiting drive output\n");
        cmd.drive = 0;
        cmd.turn = move_msg->turn;

        //If the robot was supposed to drive forward, but the path is blocked, send a replan command
        if(fabs(move_msg->turn - 0.0) < 0.01)
        {
            replan_pub.publish(obstacle_msg);
        }
    }
    //Otherwise, send the command through
    else
    {
        cmd.drive = move_msg->drive;
        cmd.turn = move_msg->turn;
    }

    //Publish the move down the chain
    move_pub.publish(cmd);
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

    //Subscribe to the navigation node for passthrough
    ros::Subscriber nav_sub = obstacle_avoid_node.subscribe(obstacle_avoid_node.resolveName("/yeet_nav/navigation"), 10, &navCallback);

    //Publish state to the obstacle topic
    obstacle_pub = obstacle_avoid_node.advertise<yeet_msgs::obstacle>(
        obstacle_avoid_node.resolveName("/yeet_msgs/obstacle"), 10);

    //Publish movement to the lower nodes
    move_pub = obstacle_avoid_node.advertise<yeet_msgs::move>(obstacle_avoid_node.resolveName("/yeet_nav/nav_cmd"), 10);

    //Replan when needed
    replan_pub = obstacle_avoid_node.advertise<yeet_msgs::obstacle>(obstacle_avoid_node.resolveName("/yeet_nav/replan"), 10);

    //Handle the callbacks
    ros::spin();
}