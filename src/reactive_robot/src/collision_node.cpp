//ROS libs and msgs
#include <ros/ros.h>


//Turtlebot libs and msgs
#include <kobuki_msgs/BumperEvent.h>


//User libs and msgs
#include <reactive_robot/collision.h>


/* Typedefs */
//Bumper state struct
typedef struct bumper_state_t
{
    bool left_bumper;
    bool center_bumper;
    bool right_bumper;
} bumper_state;


//Publishers
ros::Publisher collision_pub;

//Keep track of the bumper states
bumper_state bump_states;


/**
 * collision_callback - when collisions are detected by the bumpers, track the state of the bumpers
 * in order to halt the robot before further damage occurs.
 * 
 * @param bump_event: the message sent from the kobuki base indicating the state of a bumper
 */
void collision_callback(const kobuki_msgs::BumperEvent::ConstPtr& bump_event)
{
    //Declare the collision message as a local variable
    reactive_robot::collision collide_msg;

    //If a bumper was pressed, make sure to note that in the bumper tracking struct
    if(bump_event->state == bump_event->PRESSED)
    {
        bump_states.left_bumper = bump_event->bumper == bump_event->LEFT ? 1 : bump_states.left_bumper;
        bump_states.center_bumper = bump_event->bumper == bump_event->CENTER ? 1 : bump_states.center_bumper;
        bump_states.right_bumper = bump_event->bumper == bump_event->RIGHT ? 1 : bump_states.right_bumper;
    }
    //On the other hand, if this bumper isn't pressed, reset the appropriate flag
    else
    {
        bump_states.left_bumper = bump_event->bumper == bump_event->LEFT ? 0 : bump_states.left_bumper;
        bump_states.center_bumper = bump_event->bumper == bump_event->CENTER ? 0 : bump_states.center_bumper;
        bump_states.right_bumper = bump_event->bumper == bump_event->RIGHT ? 0 : bump_states.right_bumper;
    }

    //printf("bump: left=%d \t center=%d \t right=%d\n\r", bump_states.left_bumper, bump_states.center_bumper ,bump_states.right_bumper);

    //Collision message construction
    //If any of the flags are set to 1, we need to set the collision message flag to true
    if(bump_states.left_bumper || bump_states.center_bumper || bump_states.right_bumper)
    {
        collide_msg.collision = true;
    }
    //Otherwise, since no bumpers are pressed, there are no collisions
    else
    {
        collide_msg.collision = false;
    }

    //Publish the state of the collision detection module
    collision_pub.publish(collide_msg);
}


/**
 * Runs the loop needed to handle collision detection
 */
int main(int argc, char **argv)
{
    //Start the node
    ros::init(argc, argv, "collision_node");

    //Set up the node handle for collision detection
    ros::NodeHandle collision_node;

    //Subscribe to the bump sensors
    ros::Subscriber bump_sub = collision_node.subscribe(collision_node.resolveName("/mobile_base/events/bumper"), 10, &collision_callback);

    //Publish state to the collision topic
    collision_pub = collision_node.advertise<reactive_robot::collision>("/reactive_robot/collision", 10);

    //Handle the callbacks
    ros::spin();
}