#ifndef WORLD_MAP_NODE_H
#define WORLD_MAP_NODE_H

#include <algorithm>

//Tag constants
#define NEW     0
#define OPEN    1
#define CLOSED  2

//Set infinity
#define YEET_FINITY 1000000

class MapNode
{
    public:
        //Constructors
        MapNode();
        MapNode(int row, int col);

        //Operator overloads
        void operator=(const MapNode& node);
        bool operator==(const MapNode& node);
        bool operator>(const MapNode& left_node, const MapNode& right_node);

        //Node characteristic functions
        int getRow();
        int getCol();

        //D* search helper functions
        void reset();
        void setGoal();
        void setG(int g);
        void setGInf();
        void setMinRHS(int rhs);
        void setOpen();
        void setClosed();
        void setKeys(int primary, int secondary);
        int getG();
        int getRHS();
        int getPrimaryKey();
        bool isNew();
        bool isOpen();
        bool isGoal();
        bool isObstacle();
        //TODO: add more of these...

    private:
        //General node data
        int row;
        int col;
        int occupancy;

        //D* node data
        bool goal;
        int g_value;
        int rhs;
        int tag;
        int primary_key;
        int tiebreaker_key;
};

#endif