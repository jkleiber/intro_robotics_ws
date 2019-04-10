#include <ros/ros.h>

//Messages
#include "nav_msgs/OccupancyGrid.h"

//Libraries
#include "eigen3/Eigen/Dense.h"

//D* and Mapping classes
#include "yeet_planning/yeet_priority_queue.h"
#include "yeet_planning/map_node.h"
#include "yeet_planning/world_map.h"

//Constants
#define MAX_BUFFER 10

//Replanning flags
bool replan;

//Global map data
WorldMap current_map;

//D* variables
yeet_priority_queue<MapNode> open_list;

//Node management
MapNode goal_node;
MapNode start_node;
MapNode current_node;


//TODO: Create a service to get the next node on the path


/**
 * @brief Heuristic used to find shortest possible distance between nodes
 * This uses manhattan distance, as diagonal movement is risky in our maze environment
 * 
 * @param node_A Node A
 * @param node_B Node B
 * @return int The absolute value of the manhattan distance between two nodes
 */
int heuristic(MapNode& node_A, MapNode& node_B)
{
    return (abs(node_A.getRow() - node_B.getRow()) + abs(node_A.getCol() - node_B.getCol()));
}


void calculateKey(MapNode& node)
{
    int key1 = min(node.getG(), node.getRHS()) + heuristic(start_node, node) + km;
    int key2 = min(node.getG(), node.getRHS());

    //Set the node's new keys
    node.setKeys(key1, key2);
}


void updateVertex(MapNode& node, MapNode& prev_node)
{
    //Declare local variables
    int node_g;
    int node_rhs;

    //Initialize local variables
    node_g = node.getG();
    node_rhs = node.getRHS();

    //If the node is not a goal, update its RHS value
    if(!node.isGoal())
    {
        //Calculate the new RHS for this node
        node.setRHS(prev_node.getG() + heuristic(node, prev_node));
    }

    //If the node is on the open list, remove it from the list
    if(open_list.contains(node))
    {
        //Remove the node from open list
        open_list.remove(node);
    }

    //If the node is inconsistent, add it to the open list
    else if(node_g != node_rhs && !open_list.contains(node))
    {
        //TODO: Calculate the node's key
        

        //Add the node to the open list
        open_list.add(node);
    }
    
}

void expandNode(MapNode& node)
{
    //Declare local variables
    int i;
    std::vector<MapNode> neighbor_nodes;

    //Get neighboring nodes
    neighbor_nodes = current_map.adjacentMapNodes(node);

    //Update each neighbor node to have an rhs value of 1 more than this node
    //Note: if this were being written for 8 possible movements, 1 would be used for
    //the non-diagonal moves, while 1.4 would be used for the diagonal moves
    for(i = 0; i < neighbor_nodes.size(); ++i)
    {
        updateVertex(neighbor_nodes[i]);
    }
}


void calculateShortestPath()
{
    //Declare local variables
    MapNode node;

    //Make nodes consistent 
    while(true)//TODO: make the conditional work
    {
        //Take the node with minimum key off the open list
        node = open_list.pop();

        //If the g value is greater than the rhs, make the value over-consistent
        if(node.getG() > node.getRHS())
        {
            //Update the g-value to be over-consistent
            node.setG(node.getRHS());
            
            //Propagate changes to predecessor nodes
            expandNode(node);
        }
        else
        {
            //Set the g value to infinity
            node.setGInf();

            //Propagate changes to predecessor nodes
            expandNode(node);

            //TODO: propagate changes to this node?
        }
        
    }
}


void initSearch()
{
    //Clear all values in the map nodes
    current_map.clearParams();

    //TODO: choose the goal node somewhere
    //Set the goal node as the goal
    goal_node.setGoal();
    
    //Add the goal node to the open list
    goal_node.setOpen();
    open_list.add(goal_node);
}


void initReplan()
{

}


void planCourse()
{
    //Declare local variables
    MapNode search_node;    //node to search from

    //Initialize search
    initSearch();

    //Calculate the shortest path from goal to start
    calculateShortestPath();

    
}


//TODO: make this message a map update message (this should contain a list of cells to update and the probabilities associated with them)
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr & map_data)
{
    //TODO: update the map
    replan = true;
}


/**
 * @brief Updates the current node location using SLAM pose
 * 
 * @param odom 
 */
//TODO: get odom from the map -> odom tf thing
void updateCurrentNode(const nav_msgs::Odometry::ConstPtr &odom)
{

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
        //If the course needs to be replanned due to an unexpected event
        if(replan)
        {
            initReplan();
        }

        ros::spinOnce();
    }

    return 0;
}