#include "yeet_planning/map_cell.h"

MapNode::MapNode()
{

}


MapNode::MapNode(int row, int col)
{
    this->col = col;
    this->row = row;
}


void MapNode::operator=(const MapNode& node)
{
    this->col = node.col;
    this->g_value = node.g_value;
    this->occupancy = occupancy;
    this->primary_key = node.primary_key;
    this->rhs = node.rhs;
    this->row = node.row;
    this->tag = node.tag;
    this->tiebreaker_key = node.tiebreaker_key;
}


bool MapNode::operator==(const MapNode& node)
{
    return (this->primary_key == node.primary_key) && (this->tiebreaker_key == node.tiebreaker_key);
}


bool MapNode::operator>(const MapNode& node)
{
    //Sort by tiebreaker key if needed 
    if(node.primary_key == this->primary_key)
    {
        return this->tiebreaker_key > node.tiebreaker_key;
    }

    //Otherwise, determine the correct ordering with the primary key
    return this->primary_key > node.primary_key;
}

bool MapNode::operator<(const MapNode& right_node)
{
    //Sort by tiebreaker key if needed 
    if(right_node.primary_key == this->primary_key)
    {
        return this->tiebreaker_key < right_node.tiebreaker_key;
    }

    //Otherwise, determine the correct ordering with the primary key
    return this->primary_key < right_node.primary_key;
}

bool operator>(const MapNode& lhs, const MapNode& rhs)
{
    if(lhs.primary_key == rhs.primary_key)
    {
        return lhs.tiebreaker_key > rhs.tiebreaker_key;
    }

    return lhs.primary_key > rhs.primary_key;
}


int MapNode::getRow()
{
    return this->row;
}


int MapNode::getCol()
{
    return this->col;
}


void MapNode::setRow(int row)
{
    this->row = row;
}


void MapNode::setCol(int col)
{
    this->col = col;
}


void MapNode::reset()
{
    this->g_value = YEET_FINITY;
    this->rhs = YEET_FINITY;
    this->tag = NEW;
    this->goal = false;
}


void MapNode::setGoal()
{
    this->rhs = 0;
    this->goal = true;
}


void MapNode::setG(int g)
{
    //YEET_FINITY is the maximum value for g
    if(g > YEET_FINITY)
    {
        g = YEET_FINITY;
    }

    this->g_value = g;
}


void MapNode::setGInf()
{
    this->g_value = YEET_FINITY;
}


void MapNode::setRHS(int rhs)
{
    //YEET_FINITY is the maximum allowed value for RHS
    if(rhs > YEET_FINITY)
    {
        rhs = YEET_FINITY;
    }

    this->rhs = rhs;
}


void MapNode::setRHSInf()
{
    this->rhs = YEET_FINITY;
}


void MapNode::setOpen()
{
    this->tag = OPEN;
}


void MapNode::setClosed()
{
    this->tag = CLOSED;
}


void MapNode::setKeys(int primary, int secondary)
{
    this->primary_key = primary;
    this->tiebreaker_key = secondary;
}



void MapNode::setObstacle(bool obs)
{
    this->occupancy = obs;
}



int MapNode::getG()
{
    return this->g_value;
}


int MapNode::getRHS()
{
    return this->rhs;
}

int MapNode::getPrimaryKey()
{
    return this->primary_key;
}


bool MapNode::isNew()
{
    return (this->tag == NEW);
}


bool MapNode::isOpen()
{
    return (this->tag == OPEN);
}


bool MapNode::isGoal()
{
    return this->goal;
}


bool MapNode::isObstacle()
{
    return (occupancy >= 1);
}