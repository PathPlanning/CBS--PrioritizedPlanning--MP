#ifndef MDD_H
#define MDD_H

#include <vector>
#include <unordered_set>
#include "constraints_set.h"
#include "map.h"
#include "agent_set.h"
#include "isearch.h"
#include "astar.h"

class MDD
{
public:
    MDD();
    template<typename SearchType>
    MDD(const Map& map, const AgentSet& agentSet, SearchType* search, int agentId, int cost,
        const ConstraintsSet& constraints = ConstraintsSet());

    int getLayerSize(int cost) const;

//private:
    std::vector<int> layerSizes;
};

template <typename SearchType>
MDD::MDD(const Map& map, const AgentSet& agentSet, SearchType* search, int agentId, int cost, const ConstraintsSet& constraints) {
    Astar<> astar;
    Agent agent = agentSet.getAgent(agentId);
    Node start = agent.getStartPosition(), goal = agent.getGoalPosition();
    std::vector<std::unordered_set<Node, NodeHash>> layers;
    layers.push_back({start});

    for (int i = 0; i < cost - 1; ++i) {
        layers.push_back({});
        for (auto node : layers[i]) {
            std::list<Node> successors;
            astar.findSuccessors(successors, node, map, goal.i, goal.j, agentId, constraints);

            for (auto neigh : successors) {
                if (search->computeHFromCellToCell(neigh.i, neigh.j, goal.i, goal.j) <= cost - i - 1) {
                    layers.back().insert(neigh);
                }
            }
        }
    }

    layerSizes.resize(cost + 1, 0);
    layerSizes[cost] = 1;
    std::unordered_set<Node, NodeHash> lastLayer = {goal};
    for (int i = cost - 1; i >= 0; --i) {
        std::unordered_set<Node, NodeHash> newLastLayer;
        for (auto node : layers[i]) {
            std::list<Node> successors;
            astar.findSuccessors(successors, node, map, goal.i, goal.j, agentId, constraints);
            for (auto neigh : successors) {
                if (lastLayer.find(neigh) != lastLayer.end()) {
                    newLastLayer.insert(node);
                    break;
                }
            }
        }
        layerSizes[i] = newLastLayer.size();
        lastLayer = newLastLayer;
    }
}

#endif // MDD_H
