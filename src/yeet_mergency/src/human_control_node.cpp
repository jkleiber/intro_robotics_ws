//ROS libs and msgs
#include <ros/ros.h>

//User libs and msgs
#include <yeet_msgs/move.h>



//Publisher
ros::Publisher human_control_pub;
yeet_msgs::move move_msg;   //Published message


/* Global variables */
bool keyboard_override = false;
double keyboard_drive;
double keyboard_turn;

/**
 * @brief collision_callback - recieves instructions from collision_node and places them into a message. This data may
 * potentially be overwritten by the keyboard controls.
 * 
 * @param collision_msg: the message sent from collision_node
 */
void collision_callback(const yeet_msgs::move::ConstPtr& collision_msg)
{
    move_msg = *collision_msg;
}

/**
 * @brief keyboard_callback - recieves key values from the keyboard_node and coverts them to movement, overriding 
 * any commands from higher up the chain.
 * 
 * @param keyboard_msg: message recieved from keyboard_node
 */
void keyboard_callback(const yeet_msgs::move::ConstPtr& keyboard_move_msg)
{
    keyboard_override = true;
    keyboard_drive = keyboard_move_msg->drive;
    keyboard_turn = keyboard_move_msg->turn;
}



/**
 * Runs the loop needed to handle human control of the robot
 */
int main(int argc, char **argv)
{

    //Start the node
    ros::init(argc, argv, "human_control_node");

    //Set up the node handle for collision detection
    ros::NodeHandle human_control_node;

    //Subscribe to the bump sensors
    ros::Subscriber navigation_sub = human_control_node.subscribe(
        human_control_node.resolveName("/yeet_mergency/collision"), 10, &collision_callback);

    //TODO: Create topic between human_control_node and keyboard_node
    //Subscribe to the node directly above in the paradigm
    ros::Subscriber human_control_sub = human_control_node.subscribe(
        human_control_node.resolveName("/yeet_board/keyboard"), 10, &keyboard_callback);
    
    
    //Publish state to the collision topic
    human_control_pub = human_control_node.advertise<yeet_msgs::move>("/yeet_mergency/human_control", 10);


    //Set the loop rate of the decision function to 100 Hz
    ros::Rate loop_rate(100);

    //Make a decisision for what to do
    while(ros::ok())
    {
        //Perform all the callbacks
        ros::spinOnce();

        //TODO: All of it
        if(keyboard_override)
        {
            move_msg.drive = keyboard_drive;
            move_msg.turn = keyboard_turn;
            keyboard_override = false;
        }

        //Publish the message
        human_control_pub.publish(move_msg);
        
        //Finish the current loop
        loop_rate.sleep();
    }

}