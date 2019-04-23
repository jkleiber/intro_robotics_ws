//The algorithm for D* lite can be found at the following link:
//https://www.cs.cmu.edu/~motionplanning/lecture/AppH-astar-dstar_howie.pdf

#include <ros/ros.h>
#include <memory>

//Constants
#include "yeet_msgs/Constants.h"

//Messages
#include "nav_msgs/OccupancyGrid.h"
#include "yeet_msgs/node.h"
#include "yeet_msgs/nav_status.h"
#include "yeet_msgs/TargetNode.h"

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

//Node management
std::shared_ptr<MapNode> goal_node;
std::shared_ptr<MapNode> start_node;

//Current node check for replanning
int cur_row;
int cur_col;
int direction;

//Search state management
int search_state;

//Publishers
ros::Publisher node_pub;




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



//TODO: replace with a service instead of this status thing
void navCallback(const yeet_msgs::nav_status::ConstPtr& nav_status)
{
    if(nav_status->goal == true)
    {
        search_state = WAYPOINT;
    }
}

bool nextTargetCallback(yeet_msgs::TargetNode::Request &req, yeet_msgs::TargetNode::Response &resp)
{
    //Load the next node
    start_node = current_map.getNextWaypoint();

    //Calculate the target message
    resp.target.row = start_node->getRow();
    resp.target.col = start_node->getCol();
    resp.target.is_obstacle = start_node->isObstacle();

    //Change to the navigation system
    search_state = NAVIGATING;
    printf("GOING TO: [x: %d, y: %d]\n", start_node->getRow(), start_node->getCol());

    //Service succeeded
    return true;
}



int main(int argc, char **argv)
{
    //Initialize ROS
    ros::init(argc, argv, "d_star_node");

    //Set up the D* node
    ros::NodeHandle d_star_node;

    //Tracks obstacles
    std::shared_ptr<MapNode> obstacle_node;
    int obstacle_row;
    int obstacle_col;

    //Calculate the next target
    std::shared_ptr<MapNode> next_node;
    
    //Get data from our map when needed
    ros::Subscriber map_sub = d_star_node.subscribe(d_star_node.resolveName("/yeet_planning/map_update"), MAX_BUFFER, &mapCallback);

    //Subscribe to the task manager
    ros::Subscriber goal_sub = d_star_node.subscribe(d_star_node.resolveName("/yeet_planning/next_goal"), MAX_BUFFER, &goalCallback);

    //Manage state and publish nodes only when requested
    ros::ServiceServer target_srv = d_star_node.advertiseService(d_star_node.resolveName("/yeet_planning/target_node"), &nextTargetCallback);

    //Initialize the start node
    start_node = current_map.getNode(0, 0);

    //Wait for the task manager to tell us a goal node
    search_state = IDLE;

    //TODO: this is test code, pls remove later
    goal_node = current_map.getNode(2, 3);  //TODO: test
    search_state = NEW_GOAL;                //TODO: test
    
    while(ros::ok())
    {
        //If there is no path to goal, give up!
        if(start_node->getG() >= INFINITY)
        {
            search_state = IDLE;
            printf("D* LITE WARNING: Goal node unreachable\n");
        }

        //If the robot is in the process of navigating
        if(search_state == NAVIGATING)
        {
            //Don't do anything, let the robot drive.
        }
        //If an obstacle was found, replan
        else if(search_state == OBS_REPLAN)
        {
            //Since the obstacle is always in front, the node in front of the robot is an obstacle
            //Given the direction of the robot, determine the node that is an obstacle
            if(direction == VIEW_UP)
            {
                obstacle_row = start_node->getRow() - 1;
                obstacle_col = start_node->getCol();
            }
            else if(direction == VIEW_LEFT)
            {
                obstacle_row = start_node->getRow();
                obstacle_col = start_node->getCol() - 1;
            }
            else if(direction == VIEW_DOWN)
            {
                obstacle_row = start_node->getRow() + 1;
                obstacle_col = start_node->getCol();
            }
            else
            {
                obstacle_row = start_node->getRow();
                obstacle_col = start_node->getCol() - 1;
            }

            //Check to make sure we haven't found the wall
            if(obstacle_row < 0 || obstacle_row > (MAP_ROWS - 1) || obstacle_col < 0 || obstacle_col > (MAP_COLS - 1))
            {
                //TODO: Ignore out of bounds
                //FIXME: This is an error if this state is ever reached. We only care about obstacles that are in bounds
            }
            //Update the node that is an obstacle, assuming it is not out of bounds of the map
            else
            {
                //Update the RHS to be infinity
                obstacle_node.reset(current_map.getNode(obstacle_row, obstacle_col).get());
                obstacle_node->setRHSInf();
                obstacle_node->setObstacle(true);

                /** Update all of the directed edge costs **/
                //Go through all adjacent nodes and update the vertices
                for(int i = 0; i < 4; ++i)
                {
                    next_node = current_map.getAdjacentNode(obstacle_node, i);
                    current_map.updateVertex(next_node);
                }
                
                //Recalculate the path
                current_map.calculateShortestPath();
            }
            
        }
        //If a new goal is selected, plan a new course
        else if(search_state == NEW_GOAL)
        {
            //Plan a new course
            current_map.planCourse(goal_node);

            //Set search state to load the next waypoint
            search_state = WAYPOINT;

            current_map.printMap();
        }

        //Get the callbacks
        ros::spinOnce();
    }

    return 0;
}
