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
        std::vector<MapNode> adjacentMapNodes(int row, int col);
        //TODO: add more of these...

    private:
        //Map of nodes
        std::vector<std::vector<MapNode> > current_map;
};

#endif