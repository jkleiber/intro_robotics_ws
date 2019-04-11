#include "yeet_planning/world_map.h"


WorldMap::WorldMap()
{
    
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
            this->current_map[i][j].reset();
        }
    }
}


std::vector<MapNode>& WorldMap::adjacentMapNodes(MapNode& node)
{
    //Declare local variables
    int col;
    int row;
    std::vector<MapNode> adjacent_nodes;

    //Initialize variables
    col = node.getCol();
    row = node.getRow();

    /**
     * Check the 4 possible move directions 
     */

    //Node below
    if((row + 1) < this->current_map.size())
    {
        adjacent_nodes.push_back(this->current_map[row + 1][col]);
    }

    //Node above
    if((row - 1) >= 0)
    {
        adjacent_nodes.push_back(this->current_map[row - 1][col]);
    }

    //Node to the left
    if((col - 1) >= 0)
    {
        adjacent_nodes.push_back(this->current_map[row][col - 1]);
    }

    //Node to the right
    if((col + 1) < this->current_map[0].size())
    {
        adjacent_nodes.push_back(this->current_map[row][col + 1]);
    }

    //Return a list of the neighboring nodes
    return adjacent_nodes;
}


MapNode& WorldMap::getBestAdjNode(MapNode cur_node)
{
    //Declare local variables
    int min_value = INFINITY;
    MapNode min_node;
    std::vector<MapNode> neighbors;

    //Get the adjacent nodes
    neighbors = this->adjacentMapNodes(cur_node);

    //Find the lowest valued element
    for(int i = 0; i < neighbors.size(); ++i)
    {
        if(neighbors[i].getG() < min_value)
        {
            min_value = neighbors[i].getG();
            min_node = neighbors[i];
        }
    }

    return min_node;
}
