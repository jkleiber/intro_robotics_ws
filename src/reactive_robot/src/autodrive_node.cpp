#include <ros/ros.h>
#include <ros/console.h>

//ROS/System libs and msgs
#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

//User libs and msgs
#include "reactive_robot/drivetrain.h"
#include "reactive_robot/obstacle.h"

#define METERS_TO_FT 3.25
#define RAD_TO_DEG (double)(180.0 / 3.14159)

//Drivetrain
Drivetrain drivetrain;

//Publisher
ros::Publisher autodrive_pub;

//Global variables
geometry_msgs::Point old_pos;
geometry_msgs::Point cur_pos;

bool turning;
double current_angle;
double target_angle;
bool odometry_init;

/**
 * @brief When collisions are detected by the bumpers, track the state of the bumpers
 * in order to halt the robot before further damage occurs.
 * 
 * @param odometer: the message sent from the navigation base describing the robots odometry
 */
void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometer)
{
    //Declare local variables
    double temp1, temp2;                //Unused other than function parameters (replace with NULL?)
    double distance_traveled;           //Distance from the last turn
    
    //Messages and services
    geometry_msgs::Point new_pos;

    //Get the current position
    cur_pos.x = odometer->pose.pose.position.x;
    cur_pos.y = odometer->pose.pose.position.y;
    cur_pos.z = odometer->pose.pose.position.z;


    //Reset the old position if this is the first callback
    if(odometry_init)
    {
        old_pos = new_pos;
        odometry_init = false;
    }

    //Get the robot orientation
    tf::Pose pose;
    tf::poseMsgToTF(odometer->pose.pose, pose);

    //Get the current angle in degrees
    current_angle = drivetrain.angleWrap(tf::getYaw(pose.getRotation()) * RAD_TO_DEG);

    //Calculate the distance traveled from the fixed position
    distance_traveled = sqrt(((cur_pos.x - old_pos.x)*(cur_pos.x - old_pos.x)) + ((cur_pos.y - old_pos.y)*(cur_pos.y - old_pos.y))) * METERS_TO_FT;

    //If the distance traveled is more than a foot, turn +/- 15 degrees
    if(distance_traveled >= 1)
    {
        //Set the global varaible
        turning = true;
        
        //Generate a new turn angle
        target_angle = current_angle + ((rand() % 30) - 15);

        //Update the old position for calculating distance
        old_pos = cur_pos;
    }
}

/**
 * @brief When obstacles are detected, update old position to current position
 * 
 * @param obstacle: the created obstacle message describing obstacle state angle, distance, and drive controls
 */
void obstacleCallback(const reactive_robot::obstacle::ConstPtr& obstacle)
{
    //If there is currently an obstacle being detected
    if(obstacle->state != obstacle->EMPTY)
    {
        //Update old position
        old_pos = cur_pos;

        //Update global variable
        turning = false;
    }
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
    ros::init(argc, argv, "autodrive_node");

    //Set up the node handle for auto driving
    ros::NodeHandle autodrive_node;

    //Initialize globals
    current_angle = 0;
    turning = false;
    target_angle = 0;
    odometry_init = true;

    //Subscribe to odometry data
    ros::Subscriber odom_sub = autodrive_node.subscribe(autodrive_node.resolveName("/odom"), 10, &odometryCallback);
    ros::Subscriber obstacle_sub = autodrive_node.subscribe(autodrive_node.resolveName("reactive_robot/obstacle"), 10, &obstacleCallback);

    //Publish state to the autodrive topic
    autodrive_pub = autodrive_node.advertise<geometry_msgs::Twist>(autodrive_node.resolveName("/reactive_robot/autodrive"), 10);

    //Set the loop rate
    ros::Rate loop_rate(100);

    while(true)
    {
        //Handle the callbacks
        ros::spinOnce();

        
        //If the robot is turning to a target, turn
        if(turning)
        {
            turning = !drivetrain.turnToAngle(current_angle, target_angle);
        }
        //Otherwise drive straight
        else
        {
            drivetrain.setOutput(0.5, 0);
        }
        
        //Publish the drivetrain output
        autodrive_pub.publish(drivetrain.getOutput());        
        loop_rate.sleep();
    }
}


