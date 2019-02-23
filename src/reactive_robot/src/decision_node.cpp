//ROS msgs and libs
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//User msgs and libs
#include <reactive_robot/collision.h>
#include <reactive_robot/drivetrain.h>
#include <reactive_robot/obstacle.h>
#include <tf/transform_datatypes.h>

/* Macros and constants */
//Obstacle states
#define EMPTY       0
#define SYMMETRIC   1 
#define ASYMMETRIC  2
#define RAD_TO_DEG (double)(180.0 / 3.14159)


/* Global variables */
//Track the current state of each part of the schema
bool collide_detected;
bool forward_drive;
bool keyboard_input_detected;
bool escape_action_active;
uint8_t obstacle_type;
double turn_angle_delta;
double current_angle;
Drivetrain drivetrain;

//State data variables
geometry_msgs::Twist autodrive_output;
geometry_msgs::Twist keyboard_commands;
geometry_msgs::Twist obstacle_output;


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
void keyboardCallback(const geometry_msgs::Twist::ConstPtr& keyboard_event)
{
    keyboard_input_detected = true;
    keyboard_commands = *keyboard_event;
}


/**
 * 
 */
void obstacleCallback(const reactive_robot::obstacle::ConstPtr& obstacle_event)
{
    obstacle_type = obstacle_event->state;
    //obstacle_output = obstacle_event->drive;
}

/**
 * 
 */
void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometer)
{
    //Get the robot orientation
    tf::Pose pose;
    tf::poseMsgToTF(odometer->pose.pose, pose);

    //Get the current angle in degrees
    current_angle = drivetrain.angleWrap(tf::getYaw(pose.getRotation()) * RAD_TO_DEG);
}


/**
 * 
 */
bool twistNotZero(geometry_msgs::Twist twist)
{
    return twist.linear.x || twist.linear.y || twist.linear.z || twist.angular.x || twist.angular.y ||twist.angular.z;
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
    keyboard_input_detected = false;
    obstacle_type = EMPTY;


    //Subscribe to each of the topics published by the child nodes
    ros::Subscriber autodrive_sub = main_decision_node.subscribe(main_decision_node.resolveName("/reactive_robot/autodrive"), 10, &autodriveCallback);
    ros::Subscriber collision_sub = main_decision_node.subscribe(main_decision_node.resolveName("/reactive_robot/collision"), 10, &collisionCallback);
    ros::Subscriber keyboard_sub  = main_decision_node.subscribe(main_decision_node.resolveName("/reactive_robot/keyboard_input"), 10, &keyboardCallback);
    ros::Subscriber obstacle_sub  = main_decision_node.subscribe(main_decision_node.resolveName("/reactive_robot/obstacle"), 10, &obstacleCallback);
    ros::Subscriber odom_sub      = main_decision_node.subscribe(main_decision_node.resolveName("/odom"), 10, &odometryCallback);

    //Publish to the turtlebot's cmd_vel_mux topic
    ros::Publisher teleop_pub = main_decision_node.advertise<geometry_msgs::Twist>(main_decision_node.resolveName("/cmd_vel_mux/input/teleop"), 10);

    //Set the loop rate of the decision function to 100 Hz
    ros::Rate loop_rate(100);


    double end_angle;

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
        //Get keyboard input and output it to the turtlebot
        else if(twistNotZero(keyboard_commands))
        {
            //Set the drivetrain output to the keyboard input
            drivetrain.setOutput(keyboard_commands);

            //Indicate that the autodriving has been overridden by the user
            keyboard_input_detected = true;
        }
        //If a symmetric obstacle is detected, we need to escape 
        else if(obstacle_type == SYMMETRIC)
        {
            //If we have not already started escaping
            if(!escape_action_active)
            {
                //Set the end angle to 180 degrees away from the obstacle
                //TODO: Change current angle to the angle of the object
                end_angle = drivetrain.angleWrap(current_angle + 180);
            }
            //If we have not reached the end angle
            if(!drivetrain.turnToAngle(current_angle, end_angle))
            {
                //Continue escaping
                escape_action_active = true;
            }
            //End angle has been reached
            else
            {
                //Stop escaping
                escape_action_active = false;
            }
        }
        //If an asymmetric obstacle is detected, we need to avoid
        else if(obstacle_type == ASYMMETRIC && !escape_action_active)
        {

        }
        //The lowest priority is to drive around randomly, so do that if all other priorities are being fulfilled
        else if (!escape_action_active)
        {
            //Use the autodrive output
            drivetrain.setOutput(autodrive_output);
        }

        //Publish the desired drivetrain output to the command velocity multiplexer
        teleop_pub.publish(drivetrain.getOutput());
        
        //Make sure to limit ourselves to the loop rate
        loop_rate.sleep();
    }
}
