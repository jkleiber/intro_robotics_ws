//The algorithm for D* lite can be found at the following link:
//https://www.cs.cmu.edu/~motionplanning/lecture/AppH-astar-dstar_howie.pdf

#include <ros/ros.h>

//Constants
#include "yeet_msgs/Constants.h"

//Messages
#include "nav_msgs/OccupancyGrid.h"
#include "yeet_msgs/node.h"
#include "yeet_msgs/nav_status.h"

//Libraries
#include <eigen3/Eigen/Dense>

//D* and Mapping classes
#include "yeet_planning/yeet_priority_queue.h"
#include "yeet_planning/map_cell.h"
#include "yeet_planning/world_map.h"

//Constants
#define MAP_ROWS    yeet_msgs::Constants::MAP_ROWS
#define MAP_COLS    yeet_msgs::Constants::MAP_COLS
#define SQUARE_SIZE yeet_msgs::Constants::SQUARE_SIZE    //This is the size of the carpet squares in meters
#define MAX_BUFFER  10
#define VIEW_UP     0
#define VIEW_LEFT   1
#define VIEW_DOWN   2
#define VIEW_RIGHT  3
#define IDLE        0
#define NAVIGATING  1
#define WAYPOINT    2
#define OBS_REPLAN  3
#define NEW_GOAL    4


//Global map data
WorldMap current_map(MAP_ROWS, MAP_COLS, SQUARE_SIZE);

//D* variables
yeet_priority_queue<MapNode> open_list;

//Node management
MapNode goal_node;
MapNode start_node;

//Current node check for replanning
int cur_row;
int cur_col;
int direction;

//Search state management
int search_state;

//Publishers
ros::Publisher node_pub;



/**
 * @brief Find the cost of moving between two adjacent nodes
 * @param node_A Node A
 * @param node_B Node B
 * @return int 
 */
int transitionCost(MapNode& node_A, MapNode& node_B)
{
    //If either node is an obstacle, the transitionCost is infinity
    if(node_A.isObstacle() || node_B.isObstacle())
    {
        return YEET_FINITY;
    }

    //Otherwise the distance is 1 (nodes are adjacent and diagonal moves are not allowed)
    return 1;
}

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


int calculateKey(MapNode& node)
{
    int key1 = std::min(node.getG(), node.getRHS()) + heuristic(start_node, node);
    int key2 = std::min(node.getG(), node.getRHS());

    //Set the node's new keys
    node.setKeys(key1, key2);

    return key1;
}



void updateVertex(MapNode& node)
{
    //Declare local variables
    int i;
    MapNode neighbor_node;
    int node_g;
    int node_rhs;
    int succ_rhs;
    
    //Initialize local variables
    node_g = node.getG();
    node_rhs = node.getRHS();

    //If the node is not a goal or an obstacle, update its RHS value
    if(!node.isGoal())
    {
        //Set the RHS to infinity for comparison
        node_rhs = YEET_FINITY;

        //Find the minimum RHS for this node
        for(i = 0; i < 4; ++i)
        {
            //Get neighboring nodes
            neighbor_node = current_map.getAdjacentNode(node, i);

            //Get the RHS computed from the neighbor node
            succ_rhs = neighbor_node.getG() + transitionCost(node, neighbor_node);

            //If the RHS we just computed is lower, update the node's rhs
            if(succ_rhs < node_rhs)
            {
                node_rhs = succ_rhs;
            }
        }

        //Calculate the new RHS for this node
        node.setRHS(node_rhs);
    }

    //If the node is on the open list, remove it from the list
    if(open_list.contains(node))
    {
        //Remove the node from open list
        open_list.remove(node);
    }

    //If the node is inconsistent, add it to the open list
    if(node_g != node_rhs)
    {
        //Calculate the node's key
        calculateKey(node);

        //Add the node to the open list
        open_list.push(node);
    }
    
}


void expandNode(MapNode& node)
{
    //Declare local variables
    int i;
    MapNode neighbor_node;

    //Update each neighbor node to have an rhs value of 1 more than this node
    //Note: if this were being written for 8 possible movements, 1 would be used for
    //the non-diagonal moves, while 1.4 would be used for the diagonal moves
    for(i = 0; i < 4; ++i)
    {
        //Get a neighboring node
        neighbor_node = current_map.getAdjacentNode(node, i);

        updateVertex(neighbor_node);
    }
}


void calculateShortestPath()
{
    //Declare local variables
    MapNode node;

    node = open_list.top();

    //Make nodes consistent 
    while(node < start_node
       || start_node.getG() != start_node.getRHS())
    {
        //Take the node with minimum key off the open list
        node = open_list.top();
        open_list.pop();

        //If the g value is greater than the rhs, make the value consistent
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

            //Update this node
            updateVertex(node);
        }
        
    }
}


void initSearch()
{
    //Clear all values in the map nodes
    current_map.clearParams();

    //Set the goal node as the goal
    goal_node.setGoal();
    
    //Add the goal node to the open list
    goal_node.setOpen();
    open_list.push(goal_node);
}



void planCourse()
{
    //Initialize search
    initSearch();

    //Calculate the shortest path from goal to start
    calculateShortestPath();

    //Set search state to load the next waypoint
    search_state = WAYPOINT;
}


void goalCallback(const yeet_msgs::node::ConstPtr& goal)
{
    //Read the new node
    goal_node = current_map.getNode(goal->row, goal->col);

    //Set the search state to make a new plan
    search_state = NEW_GOAL;
}


//TODO: make this message a map update message (this should contain a list of cells to update and the probabilities associated with them)
void mapCallback(const yeet_msgs::node::ConstPtr & map_node)
{
    //Update the map


    //Set the robot to replan its route based on new environment information
    search_state = OBS_REPLAN;
}




void navCallback(const yeet_msgs::nav_status::ConstPtr& nav_status)
{
    if(nav_status->goal)
    {
        search_state = WAYPOINT;
    }
}


void getNextWaypoint()
{
    //Track the target node
    yeet_msgs::node target_node;

    //Load the next node
    start_node = current_map.getBestAdjNode(start_node);

    //Calculate the target message
    target_node.row = start_node.getRow();
    target_node.col = start_node.getCol();
    target_node.is_obstacle = start_node.isObstacle();

    //Publish the new target
    node_pub.publish(target_node);
    
    //Change to the navigation system
    search_state = NAVIGATING;
}


int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "d_star_node");

    //Set up the D* node
    ros::NodeHandle d_star_node;

    //Tracks obstacles
    MapNode obstacle_node;
    int obstacle_row;
    int obstacle_col;
    
    //Get data from our map when needed
    ros::Subscriber map_sub = d_star_node.subscribe(d_star_node.resolveName("/yeet_planning/map_update"), MAX_BUFFER, &mapCallback);

    //Subscribe to the task manager
    ros::Subscriber goal_sub = d_star_node.subscribe(d_star_node.resolveName("/yeet_planning/next_goal"), MAX_BUFFER, &goalCallback);

    //Subscribe to the navigation and obstacle avoidance to check for waypoint or Replanning
    ros::Subscriber nav_sub = d_star_node.subscribe(d_star_node.resolveName("/yeet_nav/status"), MAX_BUFFER, &navCallback);

    //Make a publisher for sending the next node's data
    node_pub = d_star_node.advertise<yeet_msgs::node>(d_star_node.resolveName("/yeet_planning/target_node"), MAX_BUFFER);

    //Initialize the start node
    start_node = current_map.getNode(0, 0);

    //Wait for the task manager to tell us a goal node
    search_state = IDLE;
    
    while(ros::ok())
    {
        //If there is no path to goal, give up!
        if(start_node.getG() >= INFINITY)
        {
            search_state = IDLE;
        }

        //If the robot is in the process of navigating
        if(search_state == NAVIGATING)
        {
            //Don't do anything, let the robot drive.
        }
        //If we need to update to go to the next waypoint, 
        //load the next node from the gradient
        else if(search_state == WAYPOINT)
        {
            //Load the next node
            getNextWaypoint();
        }
        //If an obstacle was found, replan
        else if(search_state == OBS_REPLAN)
        {
            //Since the obstacle is always in front, the node in front of the robot is an obstacle
            //Given the direction of the robot, determine the node that is an obstacle
            if(direction == VIEW_UP)
            {
                obstacle_row = start_node.getRow() - 1;
                obstacle_col = start_node.getCol();
            }
            else if(direction == VIEW_LEFT)
            {
                obstacle_row = start_node.getRow();
                obstacle_col = start_node.getCol() - 1;
            }
            else if(direction == VIEW_DOWN)
            {
                obstacle_row = start_node.getRow() + 1;
                obstacle_col =start_node.getCol();
            }
            else
            {
                obstacle_row = start_node.getRow();
                obstacle_col = start_node.getCol() - 1;
            }

            //Check to make sure we haven't found the wall
            if(obstacle_row < 0 || obstacle_row > (MAP_ROWS - 1) || obstacle_col < 0 || obstacle_col > (MAP_COLS - 1))
            {
                //TODO: Is this the right thing to do?
                //Load the next node
                getNextWaypoint();
            }
            //Update the node that is an obstacle, assuming it is not out of bounds of the map
            else
            {
                //Update the RHS to be infinity
                obstacle_node = current_map.getNode(obstacle_row, obstacle_col);
                obstacle_node.setRHSInf();
                obstacle_node.setObstacle(true);

                /** Update all of the directed edge costs **/
                //Go through all adjacent nodes and update the vertices
                for(int i = 0; i < 4; ++i)
                {
                    MapNode node = current_map.getAdjacentNode(obstacle_node, i);
                    updateVertex(node);
                }
                
                //Recalculate the path
                calculateShortestPath();
            }
            
        }
        //If a new goal is selected, plan a new course
        else if(search_state == NEW_GOAL)
        {
            //Plan a new course
            planCourse();
        }

        //Get the callbacks
        ros::spinOnce();
    }

    return 0;
}
