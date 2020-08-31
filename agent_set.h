#ifndef AGENT_SET_H
#define AGENT_SET_H

#include <vector>
#include <map>
#include <list>
#include <set>
#include <iostream>
#include "agent.h"
#include "tinyxml2.h"
#include "gl_const.h"
#include "node.h"
#include "agent_move.h"

class AgentSet
{
    private:
        std::map<std::pair<int, int>, int>      occupiedNodes;
        std::vector<Agent>                      agents;

        int rescale(int x, int scale);

    public:
        bool readAgents(const char *FileName, int scale);
        void clear();
        void addAgent(int start_i, int start_j, int goal_i, int goal_j);
        int getAgentCount() const;
        Agent getAgent(int id) const;
        int getAgentId(int i, int j) const;
        bool isOccupied(int i, int j) const;
};

#endif // AGENT_SET_H
