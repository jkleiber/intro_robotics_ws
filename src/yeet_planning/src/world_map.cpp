#include "yeet_planning/world_map.h"


WorldMap::WorldMap()
{
    
}


WorldMap::WorldMap(int rows, int cols, double square_size) : current_map(rows, std::vector<MapNode>(cols))
{
    //Declare local variables
    int c;
    int r;

    //Initialize values in the current map
    for(r = 0; r < rows; ++r)
    {
        for(c = 0; c < cols; ++c)
        {
            //Set rows and columns
            this->current_map[r][c].setRow(r);
            this->current_map[r][c].setCol(c);

            //Initialize the D* stuff
            this->current_map[r][c].reset();
        }
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
            this->current_map[i][j].reset();
        }
    }
}

MapNode& WorldMap::getAdjacentNode(MapNode& node, int idx)
{
    //Declare local variables
    int col;
    int row;

    //Initialize variables
    col = node.getCol();
    row = node.getRow();

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


MapNode& WorldMap::getBestAdjNode(MapNode& cur_node)
{
    //Declare local variables
    int min_value = YEET_FINITY;
    static MapNode min_node;
    MapNode neighbor;

    //Find the lowest valued element
    for(int i = 0; i < 4; ++i)
    {
        //Get an adjacent node
        neighbor = this->getAdjacentNode(cur_node, i);

        if(neighbor.getG() < min_value
        && neighbor.getCol() != -1)
        {
            min_value = neighbor.getG();
            min_node = neighbor;
        }
    }

    return min_node;
}

MapNode& WorldMap::getNode(int row, int col)
{
    static MapNode err_node(-1, -1);
    
    //If the node is out of bounds, return an error
    if(row < 0 || row > current_map.size() || col < 0 || col > current_map[0].size())
    {
        return err_node;
    }

    return this->current_map[row][col];
}
