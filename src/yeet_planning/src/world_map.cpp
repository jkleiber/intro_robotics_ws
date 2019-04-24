#include "yeet_planning/world_map.h"


WorldMap::WorldMap() //: current_map(yeet_msgs::Constants::MAP_ROWS, std::vector<std::shared_ptr<MapNode> >(yeet_msgs::Constants::MAP_COLS))
{
    //Declare local variables
    int c;
    int r;
    std::vector<std::shared_ptr<MapNode> > row;

    //Initialize values in the current map
    for(r = 0; r < yeet_msgs::Constants::MAP_ROWS; ++r)
    {
        for(c = 0; c < yeet_msgs::Constants::MAP_COLS; ++c)
        {
            //Set rows and columns
            std::shared_ptr<MapNode> node_ptr(new MapNode(r, c));
            node_ptr->reset();
            row.push_back(node_ptr);
        }

        this->current_map.push_back(row);
        row.clear();
    }

    start_node = current_map[0][0];
    goal_node = current_map[0][0];
}


WorldMap::WorldMap(int rows, int cols, double square_size) //: current_map(rows, std::vector<MapNode>(cols))
{
    //Declare local variables
    int c;
    int r;
    std::vector<std::shared_ptr<MapNode> > row;

    //Initialize values in the current map
    for(r = 0; r < rows; ++r)
    {
        for(c = 0; c < cols; ++c)
        {
            //Set rows and columns
            std::shared_ptr<MapNode> node_ptr(new MapNode(r, c));
            node_ptr->reset();
            row.push_back(node_ptr);
        }

        this->current_map.push_back(row);
        row.clear();
    }

    start_node = current_map[0][0];
    goal_node = current_map[0][0];
}


void WorldMap::printMap()
{
    for(int r = current_map.size() - 1; r >= 0; --r)
    {
        for(int c = current_map[0].size() - 1; c >= 0; --c)
        {
            printf("%d ", current_map[r][c]->getG());
        }
        printf("\n");
    }
}


void WorldMap::clearParams()
{
    //Declare local variables
    int i;
    int j;

    //Reset all nodes to g and rhs values of infinity
    for(i = 0; i < this->current_map.size(); ++i)
    {
        for(j = 0; j < this->current_map[i].size(); ++j)
        {
            this->current_map[i][j]->reset();
        }
    }
}


void WorldMap::setGoal(int row, int col)
{
    this->current_map[row][col]->setGoal();

    calculateKey(this->current_map[row][col]);
}


std::shared_ptr<MapNode> WorldMap::getAdjacentNode(std::shared_ptr<MapNode> node, int idx)
{
    //Declare local variables
    int col;
    int row;

    //Initialize variables
    col = node->getCol();
    row = node->getRow();

    /**
     * Check the 4 possible move directions 
     */
    switch(idx)
    {
        case 0:
            return getNode(row + 1,col);
        case 1:
            return getNode(row - 1, col);
        case 2:
            return getNode(row, col - 1);
        case 3: 
            return getNode(row, col + 1);
        default:
            return node;
    }
}


std::shared_ptr<MapNode> WorldMap::getBestAdjNode(std::shared_ptr<MapNode> cur_node)
{
    //Declare local variables
    int min_value = YEET_FINITY;
    std::shared_ptr<MapNode> min_node;
    std::shared_ptr<MapNode> neighbor;

    //If we are already at the goal, return the current node
    if(cur_node->getG() == 0)
    {
        return cur_node;
    }

    //Find the lowest valued element
    for(int i = 0; i < 4; ++i)
    {
        //Get an adjacent node
        neighbor = this->getAdjacentNode(cur_node, i);

        if(neighbor->getG() < min_value
        && neighbor->getCol() != -1)
        {
            printf("New Low!: G = %d, Row = %d, Col = %d\n", neighbor->getG(), neighbor->getRow(), neighbor->getCol());
            min_value = neighbor->getG();
            min_node = neighbor;
        }
    }

    return min_node;
}

std::shared_ptr<MapNode> WorldMap::getNode(int row, int col)
{
    std::shared_ptr<MapNode> err_ptr(new MapNode(-1, -1));
    
    //If the node is out of bounds, return an error
    if(row < 0 || row >= current_map.size() || col < 0 || col >= current_map[0].size())
    {
        return err_ptr;
    }

    return this->current_map[row][col];
}



/** D* Lite functionality **/

bool operator>(const std::shared_ptr<MapNode>& lhs, const std::shared_ptr<MapNode>& rhs)
{
    return *lhs > *rhs;
}

/**
 * @brief Find the cost of moving between two adjacent nodes
 * @param node_A Node A
 * @param node_B Node B
 * @return int 
 */
int WorldMap::transitionCost(std::shared_ptr<MapNode> node_A, std::shared_ptr<MapNode> node_B)
{
    //If either node is an obstacle, the transitionCost is infinity
    if(node_A->isObstacle() || node_B->isObstacle())
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
int WorldMap::heuristic(std::shared_ptr<MapNode> node_A, std::shared_ptr<MapNode> node_B)
{
    return (abs(node_A->getRow() - node_B->getRow()) + abs(node_A->getCol() - node_B->getCol()));
}


int WorldMap::calculateKey(std::shared_ptr<MapNode> node_ptr)
{
    int key1 = std::min(node_ptr->getG(), node_ptr->getRHS()) + heuristic(start_node, node_ptr);
    int key2 = std::min(node_ptr->getG(), node_ptr->getRHS());

    //Set the node's new keys
    node_ptr->setKeys(key1, key2);

    return key1;
}



void WorldMap::updateVertex(std::shared_ptr<MapNode> node)
{
    //Declare local variables
    int i;
    std::shared_ptr<MapNode> neighbor_node;
    int node_g;
    int node_rhs;
    int succ_rhs;
    
    //Initialize local variables
    node_g = node->getG();
    node_rhs = node->getRHS();

    //If the node is not a goal, update its RHS value
    if(!node->isGoal())
    {
        //Set the RHS to infinity for comparison
        node_rhs = YEET_FINITY;

        //Find the minimum RHS for this node
        for(i = 0; i < 4; ++i)
        {
            //Get neighboring nodes
            neighbor_node = this->getAdjacentNode(node, i);
            
            //Make sure the adjacent node is valid
            if(neighbor_node->getCol() != -1)
            {
                //Get the RHS computed from the neighbor node
                succ_rhs = neighbor_node->getG() + transitionCost(node, neighbor_node);

                //If the RHS we just computed is lower, update the node's rhs
                if(succ_rhs < node_rhs)
                {
                    node_rhs = succ_rhs;
                }
            }
        }

        //Calculate the new RHS for this node
        node->setRHS(node_rhs);
    }

    //If the node is on the open list, remove it from the list
    if(open_list.contains(*node))
    {
        //Remove the node from open list
        open_list.removeAll(*node);
    }

    //If the node is inconsistent, add it to the open list
    if(node->getG() != node->getRHS())
    {
        //Calculate the node's key
        calculateKey(node);

        //Add the node to the open list
        open_list.push(*node);
    }
    
}


void WorldMap::expandNode(std::shared_ptr<MapNode> node)
{
    //Declare local variables
    int i;
    std::shared_ptr<MapNode> neighbor_node;

    //Update each neighbor node to have an rhs value of 1 more than this node
    //Note: if this were being written for 8 possible movements, 1 would be used for
    //the non-diagonal moves, while 1.4 would be used for the diagonal moves
    for(i = 0; i < 4; ++i)
    {
        //Get a neighboring node
        neighbor_node = this->getAdjacentNode(node, i);

        //If the neighbor node is a valid node, update it
        if(neighbor_node->getCol() != -1)
        {
            updateVertex(neighbor_node);
        }
    }
}


void WorldMap::calculateShortestPath()
{
    //Declare local variables
    MapNode pop_node;
    std::shared_ptr<MapNode> node_ptr;

    //Find top value on the open list and its pointer
    pop_node = open_list.top();
    node_ptr = this->current_map[pop_node.getRow()][pop_node.getCol()];

    //Make nodes consistent 
    while((*node_ptr < *start_node || start_node->getG() != start_node->getRHS())
        && !open_list.empty())
    {
        printf("queue size: %d\n", open_list.size());
        //Take the node with minimum key off the open list
        pop_node = open_list.top();
        open_list.pop();

        //Find the pointer for this node
        node_ptr = this->current_map[pop_node.getRow()][pop_node.getCol()];
        printf("Exploring node(%d, %d) g: %d, rhs: %d, key: %d\n", node_ptr->getRow(), node_ptr->getCol(), node_ptr->getG(), node_ptr->getRHS(), node_ptr->getPrimaryKey());

        //If the g value is greater than the rhs, make the value consistent
        if(node_ptr->getG() > node_ptr->getRHS())
        {
            //Update the g-value to be over-consistent
            node_ptr->setG(node_ptr->getRHS());
            
            //Propagate changes to predecessor nodes
            expandNode(node_ptr);
        }
        else
        {
            //Set the g value to infinity
            node_ptr->setGInf();

            //Propagate changes to predecessor nodes
            expandNode(node_ptr);

            //Update this node
            updateVertex(node_ptr);
        }
        
    }
}



void WorldMap::planCourse(std::shared_ptr<MapNode> goal)
{
    /** Initialize search **/
    //Clear all values in the map nodes
    this->clearParams();

    //Set the goal node as the goal and calculate its key
    goal_node = goal;
    goal_node->setGoal();
    calculateKey(goal_node);

    //Add the goal node to the open list
    open_list.push(*goal_node);

    printf("D* Lite Initialized!\n");

    /** Search for optimal route **/
    //Calculate the shortest path from goal to start
    calculateShortestPath();
}


std::shared_ptr<MapNode> WorldMap::getNextWaypoint()
{
    //Load the next node
    start_node = this->getBestAdjNode(start_node);

    return start_node;
}
