#include <ros/ros.h>

//Scanner libs and msgs
#include <sensor_msgs/LaserScan.h>

//User libs and msgs
#include <reactive_robot/obstacle.h>

//Publisher
ros::Publisher obstacle_pub;

/**
 *TODO: isSymmetrical  -
 */
bool isSymmetrical(float start_angle, float end_angle, const sensor_msgs::LaserScan::ConstPtr& obstacle_event) 
{

}

/**
 *TODO:  calculateDistance - 
 */
float calculateDistance()
{

}
/**
 * scanCallback - Detects obstacles in front of the robot and deduces the symmetry and distance
 * of the object.
 * 
 * @param obstacle_event: the message sent from the LaserScan sensor messages containing the
 * information of the scan
 */
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& obstacle_event)
{
    //Declare the obstacle message as a local variable
    reactive_robot::obstacle obstacle_msg;

    //Start and end angles of an object
    float start_point = NULL;
    float end_point   = NULL;

    //Loop through the ranges vector at each angle
    for(int angle = 0; angle < obstacle_event->ranges.size(); ++angle)
    {
        //Determine if an obstacle is detected
        if(obstacle_event->range_min < obstacle_event->ranges[angle] && obstacle_event->range_max > obstacle_event->ranges[angle])
        {
            //If the start point has not already been set
            if(start_point == NULL)
            {
                //TODO: make sure math is correct
                start_point = (angle * obstacle_event->angle_increment) + obstacle_event->angle_min;
            }
        }
        //Else an obstacle is not detected
        else
        {
            //If the start point has been set and the end point has not been set
            if(start_point != NULL && end_point == NULL)
            {
                //TODO: make sure math is correct
                end_point = (angle * obstacle_event->angle_increment) + obstacle_event->angle_min;
            }
        }
    }
    //If and object was detected
    if(start_point != NULL) 
    {
        //Determine if it is symmetric
        bool symmetric = isSymmetrical(start_point, end_point, obstacle_event);
        obstacle_msg.distance = calculateDistance(); // TODO: parameters and resolve error
    }
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
