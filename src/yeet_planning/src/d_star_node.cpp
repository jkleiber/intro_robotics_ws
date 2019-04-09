#include <ros/ros.h>

//Messages
#include "nav_msgs/OccupancyGrid.h"

//Libraries
#include "eigen3/Eigen/Dense.h"

//Constants
#define MAX_BUFFER 10

//Replanning flags
bool replan;

//Global map data
WorldMap current_map;

void planCourse()
{

}

//TODO: make this message a map update message (this should contain a list of cells to update and the probabilities associated with them)
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & map_data)
{
    //TODO: update the map
    replan = true;
}

int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "d_star_node");

    //Set up the D* node
    ros::NodeHandle d_star_node;
    
    //Get data from our map when needed
    ros::Subscriber map_sub = d_star_node.subscribe(d_star_node.resolveName("/yeet_planning/map_update"), MAX_BUFFER, &mapCallback);

    //Set the D* to wait for the initial map before planning the course
    //TODO: we should give no initial information to the robot about its environment. It can figure out how to give the tour on its first few paths. If this doesn't work, we can give it some data
    //TODO: we definitely should give the robot information about the grid (i.e. size, number of cells, etc.) but set all occupancy probabilities to -1 (unknown) at first
    replan = false;
    
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