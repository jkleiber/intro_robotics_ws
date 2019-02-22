#include <ros/ros.h>
#include <ros/console.h>

//ROS/System libs and msgs
#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

//User libs and msgs
#include "reactive_robot/drivetrain.h"
#include <tf/transform_datatypes.h>

#define METERS_TO_FT 3.25
#define RAD_TO_DEG (double)(180.0 / 3.14159)

/* Typedefs*/
typedef struct position_state_t
{
    double x;
    double y;
    double z;
} position_state;


//Drivetrain
Drivetrain drivetrain;


//Publishers
ros::Publisher autodrive_pub;
geometry_msgs::Point old_pos;
bool turning;
double current_angle;
double target_angle;

/**
 * autodriveCallback - when collisions are detected by the bumpers, track the state of the bumpers
 * in order to halt the robot before further damage occurs.
 * 
 * @param odometer: the message sent from the navigation base describing the robots odometry
 */
void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometer)
{
    //Declare local variables
    double temp1, temp2;                //Unused other than function parameters (replace with NULL?)
    double distance_traveled;           //Distance from the last turn
    
    geometry_msgs::Point new_pos;
        
    //Get the current position
    new_pos.x = odometer->pose.pose.position.x;
    new_pos.y = odometer->pose.pose.position.y;
    new_pos.z = odometer->pose.pose.position.z;


    tf::Pose pose;
    tf::poseMsgToTF(odometer->pose.pose, pose);

    printf("old_pos.x=%f \t new_pos.x=%f\n", old_pos.x, new_pos.x);

    current_angle = drivetrain.angleWrap(tf::getYaw(pose.getRotation()) * RAD_TO_DEG);
    
    //Debug
    //printf("Current angle degrees?: %f\n", current_angle);
    
    //Calculate the distance traveled from the fixed position
    distance_traveled = sqrt(((new_pos.x - old_pos.x)*(new_pos.x - old_pos.x)) + ((new_pos.y - old_pos.y)*(new_pos.y - old_pos.y))) * METERS_TO_FT;

    //If the distance traveled is more than a foot, turn +/- 15 degrees
    if(distance_traveled >= 1)
    {
        //Set the global varaible
        turning = true;
        
        //Generate a new turn angle
        target_angle = current_angle + ((rand() % 30) - 15);

        //Update the old position for calculating distance
        old_pos = new_pos;
    }
}


/**
 * Drive the robot automatically
 */
int main(int argc, char **argv)
{
    //Start the node
    ros::init(argc, argv, "autodrive_node");

    //Set up the node handle for auto driving
    ros::NodeHandle autodrive_node;

    //Instantiate a drivetrain object for handling driving
    

    //Initialize globals
    current_angle = 0;
    turning = false;
    target_angle = 0;
    old_pos.x = 0;
    old_pos.y = 0;
    old_pos.z = 0;
    

    //Subscribe to odometry data
    ros::Subscriber odom_sub = autodrive_node.subscribe(autodrive_node.resolveName("/odom"), 10, &odometryCallback);
    
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
            drivetrain.setOutput(0.3, 0);
        }
        


        //Publish the drivetrain output
        autodrive_pub.publish(drivetrain.getOutput());        
        loop_rate.sleep();
    }
}


