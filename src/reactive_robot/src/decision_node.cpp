//ROS msgs and libs
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//User msgs and libs
#include <reactive_robot/collision.h>
#include <reactive_robot/drivetrain.h>
#include <reactive_robot/obstacle.h>

/* Macros and constants */
//Obstacle states
#define EMPTY       0
#define SYMMETRIC   1 
#define ASYMMETRIC  2


/* Global variables */
//Track the current state of each part of the schema
bool collide_detected;
bool forward_drive;
uint8_t obstacle_type;
double turn_angle_delta;

//State data variables
geometry_msgs::Twist autodrive_output;


/**
 * 
 */
void autodriveCallback(const geometry_msgs::Twist::ConstPtr& drive_event)
{
    autodrive_output = *drive_event;
}



/**
 * 
 */
void collisionCallback(const reactive_robot::collision::ConstPtr& collision_event)
{
    collide_detected = collision_event->collision;
}


/**
 * 
 */
void obstacleCallback(const reactive_robot::obstacle::ConstPtr& obstacle_event)
{
    obstacle_type = obstacle_event->state;
}



/**
 * 
 */
int main(int argc, char **argv)
{
    //Start the node
    ros::init(argc, argv, "decision_node");

    //Set up the node handle for auto driving
    ros::NodeHandle main_decision_node;

    //Initialize state trackers
    collide_detected = false;
    obstacle_type = EMPTY;

    //Initialize the motion data
    Drivetrain drivetrain;

    //Subscribe to each of the topics published by the child nodes
    ros::Subscriber collision_sub = main_decision_node.subscribe("/reactive_robot/collision", 10, &collisionCallback);
    ros::Subscriber obstacle_sub = main_decision_node.subscribe(main_decision_node.resolveName("/reactive_robot/obstacle"), 10, &obstacleCallback);
    ros::Subscriber autodrive_sub = main_decision_node.subscribe(main_decision_node.resolveName("/reactive_robot/autodrive"), 10, &autodriveCallback);

    //Publish to the turtlebot's cmd_vel_mux topic
    ros::Publisher teleop_pub = main_decision_node.advertise<geometry_msgs::Twist>(main_decision_node.resolveName("/cmd_vel_mux/input/teleop"), 10);

    //Set the loop rate of the decision function to 100 Hz
    ros::Rate loop_rate(100);

    //Given state inputs from each callback, make a decision on what to do
    while(ros::ok())
    {
        //Perform all the callbacks
        ros::spinOnce();

        //Halting after a collision is our first priority
        if(collide_detected)
        {
            drivetrain.resetOutput();
        }
        //The lowest priority is to drive around randomly, so do that if all other priorities are being fulfilled
        else
        {
            drivetrain.setOutput(autodrive_output);
        }

        //Publish the desired drivetrain output to the command velocity multiplexer
        teleop_pub.publish(drivetrain.getOutput());
        
        //Make sure to limit ourselves to the loop rate
        loop_rate.sleep();
    }
}
