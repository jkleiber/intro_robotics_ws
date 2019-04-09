#ifndef WORLD_MAP_NODE_H
#define WORLD_MAP_NODE_H

//Tag constants
#define NEW     0
#define OPEN    1
#define CLOSED  2


class MapNode
{
    public:
        //Constructors
        MapNode();
        MapNode(int row, int col);

        //Operator overloads
        void operator=(const MapNode& node);
        void operator==(const MapNode& node);
        bool operator<(const MapNode& left_node, const MapNode& right_node);

        //D* search helper functions
        //TODO: add more of these...

    private:
        //General node data
        int row;
        int col;
        int occupancy;

        //D* node data

};

#endif