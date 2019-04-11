#include "yeet_planning/map_node.h"

MapNode::MapNode()
{

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
    //TODO: implement
    return false;
}


bool MapNode::operator>(const MapNode& node)
{
    //TODO: sort by key and tiebreaker_key
    return false;
}


int MapNode::getRow()
{
    return this->row;
}


int MapNode::getCol()
{
    return this->col;
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
    this->g_value = g;
}


void MapNode::setGInf()
{
    this->g_value = YEET_FINITY;
}


void MapNode::setMinRHS(int rhs)
{
    this->rhs = std::min(this->rhs, rhs);
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