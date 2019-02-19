#include <ros/ros.h>

//ROS/System libs and msgs
#include <cstdlib>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

//User libs and msgs
#include "reactive_robot/constants.h"
#include "reactive_robot/drivetrain.h"
#include <tf/transform_datatypes.h>


/* Typedefs*/
typedef struct position_state_t
{
    double x;
    double y;
    double z;
} position_state;



//Publishers
ros::Publisher autodrive_pub;


position_state old_pos;
bool turning = false;

double current_angle;
double target_angle = 0;

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
    double distance_to_feet_factor;     //Factor used to convert ROS units into meters (unknown)
    double distance_traveled;           //Distance from the last turn
    
    position_state new_pos;             //The current position of the robot
    
    //Get the current position
    new_pos.x = odometer->pose.pose.position.x;
    new_pos.y = odometer->pose.pose.position.y;
    new_pos.z = odometer->pose.pose.position.z;

    //Get the current orientation
    tf::Quaternion q(odometer->pose.pose.orientation.x, odometer->pose.pose.orientation.y, odometer->pose.pose.orientation.z);
    tf::Matrix3x3 m(q);

    //TODO: Figure out if this is degrees or not
    m.getRPY(temp1, temp2, current_angle);
    
    //Calculate the distance traveled from the fixed position
    distance_traveled = sqrt(((new_pos.x - old_pos.x)*(new_pos.x - old_pos.x)) + (((new_pos.y - old_pos.y)*(new_pos.y - old_pos.y)))) * distance_to_feet_factor;

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
    Drivetrain drivetrain;

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
            //TODO: use a PID instead
            //If the current angle is close enough to the target angle, then stop turning
            if(abs(current_angle - target_angle) < TURN_ERROR_TOLERANCE)
            {
                turning = false;
                drivetrain.set_output(1, 0);
            }
            else if(current_angle - target_angle < 180)
            {
                drivetrain
            }
            else
            {
                
            }
            
        }
        //Otherwise drive straight
        else
        {
            drivetrain.set_output(1, 0);
        }

        //Publish the drivetrain output
        autodrive_pub.publish(drivetrain.get_output());        
    }
}


