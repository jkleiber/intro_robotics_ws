#include <ros/ros.h>

//ROS libs and msgs
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

//User libs and msgs
#include <reactive_robot/autodrive.h>

#include <cstdlib>


//Publishers
ros::Publisher autodrive_pub;

typedef struct position_state_t
{
    float64 x;
    float64 y;
    float64 z;
} position_state;


position_state old_pos;

/**
 * autodriveCallback - when collisions are detected by the bumpers, track the state of the bumpers
 * in order to halt the robot before further damage occurs.
 * 
 * @param odometer: the message sent from the navigation base describing the robots odometry
 */
void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometer)
{
    //Declare the autodrive message as a local variable
    reactive_robot::autodrive autodr_msg;

    position_state new_pos;

    new_pos.x = odometer->pose.pose.position.x;
    new_pos.y = odometer->pose.pose.position.y;
    new_pos.z = odometer->pose.pose.position.z;

    int distance_to_feet_factor;

    int distance_traveled = sqrt(((new_pos.x - old_pos.x)*(new_pos.x - old_pos.x)) + (((new_pos.y - old_pos.y)*(new_pos.y - old_pos.y))) * distance_to_feet_factor

    if(distance_traveled >= 1)
    {
        autodr_msg.drive = FALSE;
        autodr_msg.turn_angle = (rand() % 30) - 15;
    }
    else
    {
        autodr_msg.drive = TRUE;
        autodr_msg.turn_angle = 0;
    }
    
    old_pos = new_pos;

    //Publish the state of the collision detection module
    autodrive_pub.publish(autodr_msg);
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

    //TODO: Subscribe to odometry
    ros::Subscriber odom_sub = autodrive_node.subscribe(autodrive_node.resolveName("/odom"), 10, &autodriveCallback);
    
    //Publish state to the autodrive topic
    autodrive_pub = autodrive_node.advertise<reactive_robot::autodrive>(autodrive_node.resolveName("/reactive_robot/autodrive"), 10);

    //Handle the callbacks
    ros::spin();
}


