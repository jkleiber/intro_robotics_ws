#ifndef WORLD_MAP_H
#define WORLD_MAP_H

#include <vector>
#include "yeet_planning/map_node.h"

class WorldMap
{
    public:
        //Constructors
        WorldMap();
        WorldMap(int rows, int cols);

        //D* helper functions
        void clearParams();
        std::vector<MapNode>& adjacentMapNodes(MapNode node);
        
        //TODO: add more of these...

        //Navigation helper functions
        MapNode& getBestAdjNode(MapNode cur_node);

    private:
        //Map of nodes
        std::vector<std::vector<MapNode> > current_map;
};

#endif