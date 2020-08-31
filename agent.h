#ifndef AGENT_H
#define AGENT_H

#include "node.h"

class Agent
{
    private:
        int     id;
        int     start_i, start_j;
        int     goal_i, goal_j;

    public:
        Agent(int _start_i = 0, int _start_j = 0, int _goal_i = 0, int _goal_j = 0, int _id = 0) {
            start_i = _start_i;
            start_j = _start_j;
            goal_i = _goal_i;
            goal_j = _goal_j;
            id = _id;
        }
        int getStart_i() const;
        int getStart_j() const;
        int getGoal_i() const;
        int getGoal_j() const;
        int getId() const;
        Node getStartPosition() const;
        Node getGoalPosition() const;
};

#endif // AGENT_H
