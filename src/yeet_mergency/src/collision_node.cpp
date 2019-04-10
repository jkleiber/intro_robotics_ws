//ROS libs and msgs
#include <ros/ros.h>
#include <kobuki_msgs/BumperEvent.h>

//User libs and msgs
#include <yeet_msgs/move.h>


/* Typedefs */
//Bumper state struct
typedef struct bumper_state_t
{
    bool left_bumper;
    bool center_bumper;
    bool right_bumper;
} bumper_state;


//Publisher
ros::Publisher collision_pub;
yeet_msgs::move move_msg;   //Published message


/* Global variables */
bumper_state bump_states;   //Keep track of the bumper states
bool collision;             //Whether or not collision is detected


/**
 * collision_callback - when collisions are detected by the bumpers, track the state of the bumpers
 * in order to halt the robot before further damage occurs.
 * 
 * @param bump_event: the message sent from the kobuki base indicating the state of a bumper
 */
void collision_callback(const kobuki_msgs::BumperEvent::ConstPtr& bump_event)
{
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

    //Collision message construction
    //If any of the flags are set to 1, we need to set the collision message flag to true
    if(bump_states.left_bumper || bump_states.center_bumper || bump_states.right_bumper)
    {
        collision = true;
    }
    //Otherwise, since no bumpers are pressed, there are no collisions
    else
    {
        collision = false;
    }
}

/**
 * human_control_callback - when updated instructions are recieved from the human_control_node,
 * updat the message that will be sent to contain its contents.
 * 
 * @param human_control_msg: message containing the final instructions from the nodes above 
 * collision_node in the paradigm
 */
void human_control_callback(const yeet_msgs::move::ConstPtr& human_control_msg)
{
    //Simply copy the contents over
    move_msg = *human_control_msg;
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
    ros::Subscriber bump_sub = collision_node.subscribe(
        collision_node.resolveName("/mobile_base/events/bumper"), 10, &collision_callback);

    //Subscribe to the node directly above in the paradigm
    ros::Subscriber human_control_sub = collision_node.subscribe(
        collision_node.resolveName("/yeet_mergency/human_control"), 10, &human_control_callback);
    
    
    //Publish state to the collision topic
    collision_pub = collision_node.advertise<yeet_msgs::move>("/yeet_mergengy/collision", 10);


    //Set the loop rate of the decision function to 100 Hz
    ros::Rate loop_rate(100);

    //Make a decisision for what to do
    while(ros::ok())
    {
        //Perform all the callbacks
        ros::spinOnce();

        if(collision)
        {
            //TODO: Call backup-routine
            move_msg.todo = 0;
        }

        //Publish the message
        collision_pub.publish(move_msg);
        
        //Finish the current loop
        loop_rate.sleep();
    }

}