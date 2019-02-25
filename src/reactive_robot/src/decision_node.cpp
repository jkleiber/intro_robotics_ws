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
 * @brief Callback for autodrive
 * 
 * @param drive_event Event containing instructions from autodrive_node
 */
void autodriveCallback(const geometry_msgs::Twist::ConstPtr& drive_event)
{
    autodrive_output = *drive_event;
}



/**
 * @brief Callback for collision
 * 
 * @param collision_event  Event containing instructions from collision_node
 */
void collisionCallback(const reactive_robot::collision::ConstPtr& collision_event)
{
    collide_detected = collision_event->collision;
}


/**
 * @brief Callback for keyboard control
 * 
 * @param keyboard_event  Event containing instructions from the turtlebot teleop node
 */
void keyboardCallback(const geometry_msgs::Twist::ConstPtr& keyboard_event)
{    
    keyboard_commands = *keyboard_event;
}


/**
 * @brief Callback for obstacle avoidance
 * 
 * @param obstacle_event  Event containing instructions from obstacle_node
 */
void obstacleCallback(const reactive_robot::obstacle::ConstPtr& obstacle_event)
{
    obstacle_type = obstacle_event->state;
    obstacle_output = obstacle_event->drive;
}

/**
 * @brief Callback for robot position
 * 
 * @param odometer  Event containing information about the robot's orientation
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
 * @brief Check if the robot should be moving
 * 
 * @param twist Geometry message with motion information
 * @return true The robot is not moving
 * @return false The robot is moving
 */
bool twistNotZero(geometry_msgs::Twist twist)
{
    return twist.linear.x || twist.linear.y || twist.linear.z || twist.angular.x || twist.angular.y ||twist.angular.z;
}


/**
 * @brief Save the map with defined frequency
 * 
 * @param event The WTE that determines how often to refresh this
 */
void saveMap(const ros::WallTimerEvent& event)
{
    //TODO: Justin can't tell me what to do, get this path working into maps folder
    system("rosrun map_server map_saver -f ~/test_map");
}


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
            escape_action_active = false;
        }
        //Get keyboard input and output it to the turtlebot
        else if(twistNotZero(keyboard_commands))
        {
            //Set the drivetrain output to the keyboard input
            drivetrain.setOutput(keyboard_commands);
            escape_action_active = false;
        }
        //If a symmetric obstacle is detected, we need to escape 
        else if(obstacle_type == SYMMETRIC || escape_action_active)
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
            drivetrain.setOutput(obstacle_output);
        }
        //The lowest priority is to drive around randomly, so do that if all other priorities are being fulfilled
        else if (!escape_action_active)
        {
            drivetrain.setOutput(autodrive_output);
        }
        else
        {
            //Failsafe
           drivetrain.resetOutput();
        }
       
        //Publish the desired drivetrain output to the command velocity multiplexer
        teleop_pub.publish(drivetrain.getOutput());
        
        //Make sure to limit ourselves to the loop rate
        loop_rate.sleep();
    }
}
