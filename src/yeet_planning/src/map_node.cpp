#include "yeet_planning/map_node.h"

MapNode::MapNode()
{

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
    this->g_value = INFINITY;
    this->rhs = INFINITY;
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
    this->g_value = INFINITY;
}


void MapNode::setRHS(int rhs)
{
    this->rhs = rhs;
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