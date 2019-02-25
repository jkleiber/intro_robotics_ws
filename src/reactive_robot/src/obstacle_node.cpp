#include <ros/ros.h>

//Scanner libs and msgs
#include <sensor_msgs/LaserScan.h>

//User libs and msgs
#include <reactive_robot/obstacle.h>

//For isnan debug
#include<cmath>


//Constants
#define RAD_TO_DEG (double)(180.0 / 3.14159)

#define TOTAL_SAMPLES 640                                       //Laser scan samples
#define N_CENTER_SAMPLES 50                                     //Width of center view region
#define N_SIDE_SAMPLES ((TOTAL_SAMPLES - N_CENTER_SAMPLES) / 2) //Allocate the number of samples for the side views
#define LEFT_SAMPLES_IDX TOTAL_SAMPLES - N_SIDE_SAMPLES         //Where the left samples start
#define RIGHT_SAMPLES_IDX N_SIDE_SAMPLES                        //Where the right samples end
#define DETECTION_RANGE 1                                       //Range in meters to detect obstacles



//Publisher
ros::Publisher obstacle_pub;

/**
 * isSymmetrical  - Determines whether an obstacle is symmetric within a 10% tolerance
 * 
 * @param start_index: the start index of the obstacle
 * @param end_index: the end index of the obstacle
 * @param obstacle_event: the message sent from the LaserScan sensor messages containing the information of the scan
 * 
 * @return bool: returns true if the obstacle is symmetric and false if the object is asymmetric
 */
bool isSymmetrical(int start_index, int end_index, const sensor_msgs::LaserScan::ConstPtr& obstacle_event) 
{
    //Set the return boolean to true to start
    bool symmetric = true;
    //Find the middle index of the obstalce
    int mid_index = (start_index + end_index) / 2;
    //Loop from left side and right side and cross check distances
    for(int left = start_index; left < mid_index; ++left)
    {
        for(int right = end_index; right > mid_index; --right)
        {
            //Assign the distance to variables so ranges is only called once for each side respectively
            float left_distance = obstacle_event->ranges[left];
            float right_distance = obstacle_event->ranges[right];
            //If the right distance is outside of left distance with a 10% tolerance, set symmetric to false and break 
            if((left_distance + left_distance * 0.05) > right_distance || (left_distance - left_distance * 0.05) < right_distance)
            {
                symmetric = false;
                break;
            }
        }
    }
    return symmetric;
}

/**
 * calculateDistance - Calculates the closest distance of the obstacle
 */
float calculateDistance(int start_index, int end_index, const sensor_msgs::LaserScan::ConstPtr& obstacle_event)
{
    //Set closest disance to 1 meter, as all distances of the object will be closer (i.e. 1 foot or less)
    float closest_distance = 1;
    for(start_index; start_index <= end_index; ++start_index)
    {
        closest_distance = (obstacle_event->ranges[start_index] < closest_distance) ? obstacle_event->ranges[start_index] : closest_distance;
    }
    return closest_distance;
}
/**
 * scanCallback - Detects obstacles in front of the robot and deduces the symmetry and distance
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

    //Loop through all the samples
    for(int i = 0; i < TOTAL_SAMPLES; ++i)
    {
        //Right samples
        if(i < RIGHT_SAMPLES_IDX)
        {
            if(!std::isnan(obstacle_event->ranges[i]) && obstacle_event->ranges[i] <= distance_to_right_obj)
            {
                distance_to_right_obj = obstacle_event->ranges[i];
            }
        }
        //Center region
        else if(i >= RIGHT_SAMPLES_IDX && i < LEFT_SAMPLES_IDX)
        {
            if(!std::isnan(obstacle_event->ranges[i]) && obstacle_event->ranges[i] <= distance_to_center_obj)
            {
                distance_to_center_obj = obstacle_event->ranges[i];
                symmetric_obj_index = i;
            }
        }
        //Left region
        else
        {
            if(!std::isnan(obstacle_event->ranges[i]) && obstacle_event->ranges[i] <= distance_to_left_obj)
            {
                distance_to_left_obj = obstacle_event->ranges[i];
            }
        }
        
    }


    if(distance_to_left_obj > DETECTION_RANGE && distance_to_center_obj > DETECTION_RANGE && distance_to_right_obj > DETECTION_RANGE)
    {
        obstacle_msg.state = obstacle_msg.EMPTY;
        obstacle_msg.drive.angular.z = 0;
    }
    else if(distance_to_center_obj <= DETECTION_RANGE)
    {
        obstacle_msg.state = obstacle_msg.SYMMETRIC;
        obstacle_msg.angle = ((symmetric_obj_index * obstacle_event->angle_increment) + obstacle_event->angle_min) * RAD_TO_DEG;
    }
    else
    {
        obstacle_msg.state = obstacle_msg.ASYMMETRIC;
        obstacle_msg.drive.linear.x = 0;
        obstacle_msg.drive.linear.y = 0;
        obstacle_msg.drive.linear.z = 0;
        

        if(distance_to_left_obj < distance_to_right_obj)
        {
            obstacle_msg.drive.angular.z = -0.4;
        }
        else if (distance_to_left_obj > distance_to_right_obj)
        {
            obstacle_msg.drive.angular.z = 0.4;
        }
        else
        {
            obstacle_msg.drive.angular.z = 0;
        }
    }

    //Publish the instructions from this node
    obstacle_pub.publish(obstacle_msg);
}
/**
 * Runs the loop needed to handle obstacle detection 
 */
int main(int argc, char **argv)
{
    //Start the node
    ros::init(argc, argv, "obstacle_avoid_node");

    //Set up the node handle for auto driving
    ros::NodeHandle obstacle_avoid_node;

    //Subscribe to the scanner
    ros::Subscriber obstacle_sub = obstacle_avoid_node.subscribe(obstacle_avoid_node.resolveName("/scan"), 10, &scanCallback);

    //Publish state to the obstacle topic
    obstacle_pub = obstacle_avoid_node.advertise<reactive_robot::obstacle>(obstacle_avoid_node.resolveName("/reactive_robot/obstacle"), 10);

    //Handle the callbacks
    ros::spin();
}
