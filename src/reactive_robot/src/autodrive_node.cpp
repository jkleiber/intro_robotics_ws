#include <ros/ros.h>

//ROS libs and msgs
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//User libs and msgs
#include <reactive_robot/autodrive.h>

#include <cstdlib>


//Publishers
ros::Publisher autodrive_pub;

typedef struct position_state_t
{
    double x;
    double y;
    double z;
} position_state;

//
position_state old_pos;

/**
 * autodriveCallback - when collisions are detected by the bumpers, track the state of the bumpers
 * in order to halt the robot before further damage occurs.
 * 
 * @param odometer: the message sent from the navigation base describing the robots odometry
 */
void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometer)
{
    //Declare local variables
    double distance_to_feet_factor;
    double distance_traveled;
    position_state new_pos;
    
    //Get the current position
    new_pos.x = odometer->pose.pose.position.x;
    new_pos.y = odometer->pose.pose.position.y;
    new_pos.z = odometer->pose.pose.position.z;

    //Calculate the distance traveled from the fixed position
    distance_traveled = sqrt(((new_pos.x - old_pos.x)*(new_pos.x - old_pos.x)) + (((new_pos.y - old_pos.y)*(new_pos.y - old_pos.y)))) * distance_to_feet_factor;

    //If the distance traveled is more than a foot, turn +/- 15 degrees
    if(distance_traveled >= 1)
    {
        //

        //
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

        //
    }
}


