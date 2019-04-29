//ROS libs and msgs
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Empty.h>

//User msgs and libs
#include <yeet_nav/pid_controller.h>
#include "yeet_msgs/Constants.h"
#include "yeet_msgs/nav_status.h"
#include "yeet_msgs/move.h"
#include "yeet_msgs/node.h"
#include "yeet_msgs/obstacle.h"
#include "yeet_msgs/TargetNode.h"

//Constants
#define RAD_TO_DEG (double)(180.0 / 3.14159)    //Conversion factor from radians to degrees
#define DISTANCE_TOL (double)(0.075)            //Tolerance for drive distance
#define ANGLE_TOL (double)(1.0)                 //Tolerance for angle distance
#define UP 0                                    //Up map angle
#define LEFT 90                                 //Left map angle
#define RIGHT -90                               //Right map angle
#define DOWN 180                                //Down map angle
#define NAV_BUFFER (double)(0.125)               //Tiny buffer added to get to a square center

//Drive X PID
#define X_KP (double)(0.6)
#define X_KI (double)(0.002)
#define X_KD (double)(0.001)
#define X_MAX_OUTPUT 0.3
#define X_MIN_OUTPUT -0.3

//Drive Y PID
#define Y_KP (double)(0.6)
#define Y_KI (double)(0.002)
#define Y_KD (double)(0.01)
#define Y_MAX_OUTPUT 0.3
#define Y_MIN_OUTPUT -0.3

//Turn PID
#define TURN_KP (double)(0.2)
#define TURN_KI (double)(0.000)
#define TURN_KD (double)(0.001)
#define TURN_MAX_OUTPUT 0.5
#define TURN_MIN_OUTPUT -0.5

//Ramping 
double sec_last_out;
double last_output;

//PID
PID_Controller turn;
PID_Controller drive_x;
PID_Controller drive_y;

//Global variables
yeet_msgs::Constants constants;
double current_angle;
int map_angle;
int goal_row;
int goal_col;
int last_goal_row;
int last_goal_col;
int cur_row;
int cur_col;
int row_diff;
int col_diff;
double x;
double y;
double goal_x;
double goal_y;
bool drive_enabled;

//ROS service
ros::ServiceClient target_srv;
yeet_msgs::TargetNode target_node;

//ROS Publishers
ros::Publisher obstacle_pub;


/**
 * @brief angleWrap - Keep angles within the expected range
 * 
 * @param angle - Unwrapped angle
 * @return double - Angle between 0-360
 */
double angleWrap(double angle)
{
    return angle < 0 ? fmod(angle, 360) + 360 : fmod(angle, 360);
}



/**
 * @brief goalCallBack- Updates global variables for the PID Controller to use.
 * 
 * @param goal - The information about the robot's goal
 */
void goalCallBack(const yeet_msgs::node goal)
{
    drive_x.reset(x);
    drive_y.reset(y);
    turn.reset(current_angle);
    goal_row = goal.row;
    goal_col = goal.col;
    goal_x = ((double)(goal_row) * yeet_msgs::Constants::SQUARE_SIZE) + NAV_BUFFER;
    goal_y = ((double)(goal_col) * yeet_msgs::Constants::SQUARE_SIZE) + NAV_BUFFER;

    //Get the difference in rows and columns
    col_diff = last_goal_col - goal_col;
    row_diff = last_goal_row - goal_row;

    printf("last row: %d last col: %d\n", last_goal_row, last_goal_col);

    map_angle = 0;//current_angle;
    //Down a square
    map_angle = (row_diff == 0 && col_diff == 1) ? RIGHT : map_angle; 
    //Right a sqaure
    map_angle = (row_diff == 1 && col_diff == 0) ? DOWN : map_angle;
    //Up a square
    map_angle = (row_diff == 0 && col_diff == -1) ? LEFT : map_angle;
    //Left a square
    map_angle = (row_diff == -1 && col_diff == 0) ? UP : map_angle;

    printf("NAV_NODE: Received command to move to row: %d col: %d angle:%d\n", goal_row, goal_col, map_angle);

    //Set the last goal row and column
    last_goal_col = goal_col;
    last_goal_row = goal_row;
}

/**
 * @brief odomCallBack - Gets the current angle of the robot in degrees
 * 
 * @param odom - The odometry message containing robot angle position
 */
void odomCallBack(const nav_msgs::Odometry::ConstPtr& odom)
{
    //Get the robot orientation
    tf::Pose pose;
    tf::poseMsgToTF(odom->pose.pose, pose);

    //Get the current angle in degrees
    current_angle = angleWrap(tf::getYaw(pose.getRotation()) * RAD_TO_DEG);

    //printf("current_angle: %f\n", current_angle);

    //Get the current row and column from x and y position
    x = odom->pose.pose.position.x;
    y = odom->pose.pose.position.y;
    cur_col = (int) round(y / constants.SQUARE_SIZE);
    cur_row = (int) round(x / constants.SQUARE_SIZE);
}


void replanCallback(const yeet_msgs::obstacle::ConstPtr& obstacle_msg)
{
    //Declare local variables
    yeet_msgs::node obstacle_node;

    if(obstacle_msg->obstacle && drive_enabled)
    {
        drive_enabled = false;
        last_goal_col = cur_col;
        last_goal_row = cur_row;

        //Publish to D* to alert need to replan, given the goal node is an obstacle
        obstacle_node.col = cur_col;
        obstacle_node.row = cur_row;
        obstacle_node.is_obstacle = true;

        printf("REPLAN: Current node is: %d, %d\n", cur_row, cur_col);

        obstacle_pub.publish(obstacle_node);
    }
}


void enableDriveCallback(const std_msgs::Empty::ConstPtr& empty)
{
    drive_enabled = true;
    
    if(target_srv.call(target_node))
    {
        goalCallBack(target_node.response.target);
    }
}


/**
 * @brief sweep - 
 * 
 * @param target_angle - The desired turn angle
 * @return double - Returns the error in angle. Negative if the turn
 * should be to the right, positive if left.
 */
double sweep(double target_angle)
{
    double sweep = target_angle - current_angle;
    sweep = (sweep >  180) ? sweep - 360 : sweep;
    sweep = (sweep < -180) ? sweep + 360 : sweep;
    return sweep;
}

/**
 * @brief - Main method
 * 
 * @param argc - Number of args
 * @param argv - Args into the executable
 * @return int - Exit code
 */
int main(int argc, char **argv)
{
    //Start the node
    ros::init(argc, argv, "nav_node");

    //Set up the node to handle motion
    ros::NodeHandle nav_node;
    
    //Initialize default location values
    current_angle = 0.0;
    x = 0.0;
    y = 0.0;
    cur_row = 0;
    cur_col = 0;
    goal_row = 0;
    goal_col = 0;
    last_goal_row = 0;
    last_goal_col = 0;
    last_output = 0;
    sec_last_out = 0;
    drive_enabled = true;

    //Initialize PID
    drive_x.init(X_KP, X_KI, X_KD, X_MAX_OUTPUT, X_MIN_OUTPUT);
    drive_y.init(Y_KP, Y_KI, Y_KD, Y_MAX_OUTPUT, Y_MIN_OUTPUT);
    turn.init(TURN_KP, TURN_KI, TURN_KD, TURN_MAX_OUTPUT, TURN_MIN_OUTPUT);

    //Subscribe to topics
    ros::Subscriber odom_sub = nav_node.subscribe(
        nav_node.resolveName("/odom"), 10, &odomCallBack);
    
    ros::Subscriber replan_sub = nav_node.subscribe(nav_node.resolveName("/yeet_nav/replan"), 10, &replanCallback);
    ros::Subscriber enable_drive_sub = nav_node.subscribe(nav_node.resolveName("/yeet_nav/enable_drive"), 10, &enableDriveCallback);

    //Publishers
    ros::Publisher move_pub = nav_node.advertise<yeet_msgs::move>(
        nav_node.resolveName("/yeet_nav/navigation"), 10);
    
    obstacle_pub = nav_node.advertise<yeet_msgs::node>(nav_node.resolveName("/yeet_planning/map_update"), 10);

    //Service for requesting new target_node
    target_srv = nav_node.serviceClient<yeet_msgs::TargetNode>(nav_node.resolveName("/yeet_planning/target_node"));

    //Set the loop rate of the nav function to 100 Hz
    ros::Rate loop_rate(100);

    //Create local messages
    yeet_msgs::move move;

    //The callback and logic loop
    while(ros::ok())
    {
        //Initialize to zero
        move.drive = 0;
        move.turn = 0;

        //Only drive if driving is encabled
        if(drive_enabled)
        {
            //Within tolerance, stop turning and start driving
            if(abs(sweep((double)(map_angle))) <= ANGLE_TOL)
            {
                //printf("X ERR: %f, Y ERR: %f, ANG: %d\n", fabs(x - goal_x), fabs(y - goal_y), map_angle);
                
                if((fabs(x - goal_x) < DISTANCE_TOL && (map_angle == DOWN || map_angle == UP)) 
                || (fabs(y - goal_y) < DISTANCE_TOL && (map_angle == LEFT || map_angle == RIGHT)))
                {
                    //We have reached the goal, so get a new node from the D*
                    if(target_srv.call(target_node))
                    {
                        goalCallBack(target_node.response.target);
                    }
                    //Otherwise notify there was an error
                    else
                    {
                        //printf("NAV_NODE ERROR: Service call to D* Lite failed!\n");
                    }
                }
                else
                {
                    //printf("YEET 2 %f vs %f @ %d \n", x, goal_x, map_angle);
                    move.drive = (map_angle == DOWN || map_angle == UP) ? fabs(drive_x.getOutput(goal_x, x)) : move.drive;
                    move.drive = (map_angle == LEFT || map_angle == RIGHT) ? fabs(drive_y.getOutput(goal_y, y)) : move.drive;
                }

            }
            //Keep turning and do not drive
            else
            {
                printf("ERR: %f\n", sweep((double)(map_angle)));
                move.turn = -turn.getOutput(0, sweep((double)(map_angle)));
            }

            //Ramp up and down
            move.drive = (move.drive + last_output + sec_last_out) / 3.0;
            sec_last_out = last_output;
            last_output = move.drive;

            //Publish
            move_pub.publish(move);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}