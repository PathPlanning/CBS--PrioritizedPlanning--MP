#include "agent.h"


int Agent::getStart_i() const
{
    return start_i;
}

int Agent::getStart_j() const
{
    return start_j;
}

int Agent::getGoal_i() const
{
    return goal_i;
}

int Agent::getGoal_j() const
{
    return goal_j;
}

int Agent::getId() const
{
    return id;
}

Node Agent::getStartPosition() const {
    return Node(start_i, start_j);
}

Node Agent::getGoalPosition() const {
    return Node(goal_i, goal_j);
}


