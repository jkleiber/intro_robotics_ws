//ROS libs and msgs
#include <ros/ros.h>

//User libs and msgs
#include <yeet_motion/drivetrain.h>

Drivetrain drivetrain;

void movementCallback()

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
    ros::init(argc, argv, "motion_node");

    //Set up the node to handle motion
    ros::NodeHandle motion_node;

    //Subscribe to topics
    ros::Subscriber movement_sub = motion_node.subscribe(
        motion_node.resolveName("/yeet_mergency/movement"), 10, &movementCallback);

    //Publish to the turtlebot's cmd_vel_mux topic
    ros::Publisher teleop_pub = main_decision_node.advertise<geometry_msgs::Twist>(
        main_decision_node.resolveName("/cmd_vel_mux/input/teleop"), 10);
}