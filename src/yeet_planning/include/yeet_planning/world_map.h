#ifndef WORLD_MAP_H
#define WORLD_MAP_H

#include <vector>
#include "yeet_planning/map_node.h"

class WorldMap
{
    public:
        //Constructors
        WorldMap();
        WorldMap(int rows, int cols, double square_size);

        //Map functions
        //TODO: make this more usable by people that don't just have constants at the top of their calling file

        //D* helper functions
        void clearParams();
        std::vector<MapNode>& adjacentMapNodes(MapNode& node);
        
        //Getters for nodes
        MapNode& getNode(int row, int col);

        //Navigation helper functions
        MapNode& getBestAdjNode(MapNode cur_node);

    private:
        //Map of nodes
        std::vector<std::vector<MapNode> > current_map;

        int num_rows;
        int num_cols;
};

#endif