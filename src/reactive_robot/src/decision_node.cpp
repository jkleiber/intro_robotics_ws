//ROS msgs and libs
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//User msgs and libs
#include <reactive_robot/autodrive.h>
#include <reactive_robot/constants.h>
#include <reactive_robot/collision.h>
#include <reactive_robot/obstacle.h>


//Track the current state of each part of the schema
bool collide_detected;
bool forward_drive;
uint8_t obstacle_type;
double turn_angle_delta;

//State data variables
double start_angle;
bool auto_turning;


/**
 * 
 */
void autodriveCallback(const reactive_robot::autodrive::ConstPtr& drive_event)
{
    forward_drive = drive_event->drive;
    turn_angle_delta = drive_event->turn_angle;
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
 * zero_output - sets all Twist outputs to 0
 */
void zero_output(geometry_msgs::Twist& cur_output)
{
    cur_output.angular.x = 0;
    cur_output.angular.y = 0;
    cur_output.angular.z = 0;
    cur_output.linear.x = 0;
    cur_output.linear.y = 0;
    cur_output.linear.z = 0;
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

    //Initialize the state data
    auto_turning = false;

    //Initialize the motion data
    geometry_msgs::Twist current_output;
    zero_output(current_output);

    //Subscribe to each of the topics published by the child nodes
    ros::Subscriber collision_sub = main_decision_node.subscribe(main_decision_node.resolveName("/reactive_robot/collision"), 10, &collisionCallback);
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
            //Make the turtlebot halt
            zero_output(current_output);

            //Publish the stop command
            teleop_pub.publish(current_output);
        }
        //The lowest priority is to drive around randomly, so do that if all other priorities are being fulfilled
        else
        {
            //Reset the output
            zero_output(current_output);

            //If driving forward, drive forward
            if(forward_drive)
            {
                current_output.linear.x = 1;
            }
            //Turn if it has been a foot. Aim for the random target angle
            else
            {
                
            }
            
        }
        

        //Make sure to limit ourselves to the loop rate
        loop_rate.sleep();
    }
}
