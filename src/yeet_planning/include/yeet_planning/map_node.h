#ifndef WORLD_MAP_NODE_H
#define WORLD_MAP_NODE_H


class MapNode
{
    public:
        //Constructors
        MapNode();
        MapNode(int row, int col);

        //Operator overloads
        void operator=(const MapNode& node);
        void operator==(const MapNode& node);

        //D* search helper functions
        //TODO: add more of these...

    private:
        //General node data
        int row;
        int col;
        int occupancy;

        //D* node data
        double path_cost;
        double cost_to_go;
        double eval_cost;

        //Backpointer data
        int bp_row;
        int bp_col;
};

#endif