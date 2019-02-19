//ROS msgs and libs
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

//User msgs and libs
#include <reactive_robot/collision.h>


//Track the current state of each part of the schema
bool collide_detected;


//Current output to the TurtleBot
geometry_msgs::Twist current_output;


void collisionCallback(const reactive_robot::collision::ConstPtr& collision_event)
{
    collide_detected = collision_event->collision;
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

    //Subscribe to each of the topics published by the child nodes
    ros::Subscriber collision_sub = main_decision_node.subscribe(main_decision_node.resolveName("/reactive_robot/collision"), 10, &collisionCallback);

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
            current_output.angular.x = 0;
            current_output.angular.y = 0;
            current_output.angular.z = 0;
            current_output.linear.x = 0;
            current_output.linear.y = 0;
            current_output.linear.z = 0;

            //Publish the stop command
            teleop_pub.publish(current_output);
        }

        //Make sure to limit ourselves to the loop rate
        loop_rate.sleep();
    }
}
