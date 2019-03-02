//ROS msgs and libs
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


//User libs and msgs
#include <reactive_robot/obstacle.h>


//Other libs
#include <cmath>
#include <math.h>


//Constants
#define RAD_TO_DEG (double)(180.0 / 3.14159)                    //Radians to Degrees
#define FT_TO_METERS (double)(1.0 / 3.25)                       //Feet to Meters
#define TOTAL_SAMPLES 640                                       //Laser scan samples
#define N_CENTER_SAMPLES 75                                     //Width of center view region
#define N_SIDE_SAMPLES ((TOTAL_SAMPLES - N_CENTER_SAMPLES) / 2) //Allocate the number of samples for the side views
#define LEFT_SAMPLES_IDX TOTAL_SAMPLES - N_SIDE_SAMPLES         //Where the left samples start
#define RIGHT_SAMPLES_IDX N_SIDE_SAMPLES                        //Where the right samples end
#define DETECTION_RANGE (double)(2.5 * FT_TO_METERS)            //Range in meters to detect obstacles

//Publisher
ros::Publisher obstacle_pub;

/**
 * @brief Detects obstacles in front of the robot and deduces the symmetry and distance
 * of the object.
 * 
 * @param obstacle_event: the message sent from the LaserScan sensor messages containing the information of the scan
 */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& obstacle_event)
{
    //Declare the obstacle message as a local variable
    reactive_robot::obstacle obstacle_msg;

    //Start and end indices of an object
    int symmetric_obj_index = -1;

    //Store the samples in three nice arrays
    double left_scan[N_SIDE_SAMPLES];
    double center_scan[N_CENTER_SAMPLES];
    double right_scan[N_SIDE_SAMPLES];
    
    //Track distance to nearest object in each view region
    double distance_to_left_obj = INT_MAX;
    double distance_to_center_obj = INT_MAX;
    double distance_to_right_obj = INT_MAX;

    //Loop through all the samples and update closest distance from objects in each region if needed
    for(int i = 0; i < TOTAL_SAMPLES; ++i)
    {
        //Right region
        if(i < RIGHT_SAMPLES_IDX)
        {
            //If the distance to the right obstacle is less than the current closest distance
            if(!std::isnan(obstacle_event->ranges[i]) && obstacle_event->ranges[i] <= distance_to_right_obj)
            {
                //Update closest distance to right object
                distance_to_right_obj = obstacle_event->ranges[i];
            }
        }
        //Center region
        else if(i >= RIGHT_SAMPLES_IDX && i < LEFT_SAMPLES_IDX)
        {
             //If the distance to the center obstacle is less than the current closest distance
            if(!std::isnan(obstacle_event->ranges[i]) && obstacle_event->ranges[i] <= distance_to_center_obj)
            {
                //Update closest distance to cetner object
                distance_to_center_obj = obstacle_event->ranges[i];
            }
        }
        //Left region
        else
        {
             //If the distance to the left obstacle is less than the current closest distance
            if(!std::isnan(obstacle_event->ranges[i]) && obstacle_event->ranges[i] <= distance_to_left_obj)
            {
                //Update closest distance to left object
                distance_to_left_obj = obstacle_event->ranges[i];
            }
        }
        
    }

    //If no obstacle is detected
    if(distance_to_left_obj > DETECTION_RANGE && distance_to_center_obj > DETECTION_RANGE && 
        distance_to_right_obj > DETECTION_RANGE)
    {
        //Update stae to empty
        obstacle_msg.state = obstacle_msg.EMPTY;
        //Reset angular velocity
        obstacle_msg.drive.angular.z = 0;
    }
    //If an object directly in front of the object is detected
    else if(distance_to_center_obj <= DETECTION_RANGE || distance_to_left_obj == distance_to_right_obj)
    {
        //Update state to symmetric
        obstacle_msg.state = obstacle_msg.SYMMETRIC;
        //Update obstacles angle from robot
        obstacle_msg.angle = ((symmetric_obj_index * obstacle_event->angle_increment) + 
            obstacle_event->angle_min) * RAD_TO_DEG;
    }
    //Otherwise the object is in the left or right region
    else
    {
        //Update message state and linear velocities
        obstacle_msg.state = obstacle_msg.ASYMMETRIC;
        obstacle_msg.drive.linear.x = 0.1;
        obstacle_msg.drive.linear.y = 0;
        obstacle_msg.drive.linear.z = 0;
        
        //If the left object is closer, turn right
        if(distance_to_left_obj < distance_to_right_obj)
        {
            obstacle_msg.drive.angular.z = -0.6;
        }
        //If the right object is closer, turn left
        else
        {
            obstacle_msg.drive.angular.z = 0.6;
        }
    }

    //Publish the instructions from this node
    obstacle_pub.publish(obstacle_msg);
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
    ros::init(argc, argv, "obstacle_avoid_node");

    //Set up the node handle for auto driving
    ros::NodeHandle obstacle_avoid_node;

    //Subscribe to the scanner
    ros::Subscriber obstacle_sub = obstacle_avoid_node.subscribe(
        obstacle_avoid_node.resolveName("/scan"), 10, &scanCallback);

    //Publish state to the obstacle topic
    obstacle_pub = obstacle_avoid_node.advertise<reactive_robot::obstacle>(
        obstacle_avoid_node.resolveName("/reactive_robot/obstacle"), 10);

    //Handle the callbacks
    ros::spin();
}
