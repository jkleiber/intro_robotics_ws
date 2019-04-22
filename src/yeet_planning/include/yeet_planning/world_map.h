#ifndef WORLD_MAP_H
#define WORLD_MAP_H

#include <memory>
#include <vector>
#include "yeet_planning/map_cell.h"
#include "yeet_planning/yeet_priority_queue.h"
#include "yeet_msgs/Constants.h"
#include "yeet_msgs/node.h"

class WorldMap
{
    public:
        //Constructors
        WorldMap();
        WorldMap(int rows, int cols, double square_size);

        //Map functions
        void printMap();

        //D* helper functions
        void clearParams();
        void setGoal(int row, int col);
        int transitionCost(std::shared_ptr<MapNode> node_A, std::shared_ptr<MapNode> node_B);
        int heuristic(std::shared_ptr<MapNode> node_A, std::shared_ptr<MapNode> node_B);
        int calculateKey(std::shared_ptr<MapNode> node_ptr);
        void updateVertex(std::shared_ptr<MapNode> node);
        void expandNode(std::shared_ptr<MapNode> node);
        void calculateShortestPath();
        void planCourse(std::shared_ptr<MapNode> goal);
        std::shared_ptr<MapNode> getNextWaypoint();
        
        //Getters for nodes
        std::shared_ptr<MapNode> getNode(int row, int col);
        std::shared_ptr<MapNode> getAdjacentNode(std::shared_ptr<MapNode> node, int idx);

        //Navigation helper functions
        std::shared_ptr<MapNode> getBestAdjNode(std::shared_ptr<MapNode> cur_node);

    private:
        //Map of nodes
        std::vector<std::vector<std::shared_ptr<MapNode> > > current_map;

        int num_rows;
        int num_cols;

        //D* variables
        yeet_priority_queue<MapNode> open_list;
        std::shared_ptr<MapNode> start_node;
        std::shared_ptr<MapNode> goal_node;
};

#endif