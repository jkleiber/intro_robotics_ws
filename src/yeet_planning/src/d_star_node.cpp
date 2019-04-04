#include <ros/ros.h>

//Replanning flags
bool replan;

void planCourse()
{

}

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "d_star_node");

    //Set up the D* node
    ros::NodeHandle d_star_node;
    
    //Get data from occupancy grid

    //Set the D* to plan an initial course
    replan = true;
    
    while(ros::ok())
    {
        if(replan)
        {
            planCourse();
        }

        ros::spinOnce();
    }

    return 0;
}