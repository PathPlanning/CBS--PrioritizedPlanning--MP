#ifndef MPCONFLICTBASEDSEARCH_H
#define MPCONFLICTBASEDSEARCH_H

#include "conflict_based_search.h"
#include "motion_primitives.h"

template <typename SearchType = TwoKNeighSIPP<>>
class MPConflictBasedSearch : public ConflictBasedSearch<SearchType>
{
    public:
        MPConflictBasedSearch();
        MPConflictBasedSearch(SearchType* Search) :
            ConflictBasedSearch<SearchType>(Search) {
            this->mp = Search->getMP();
        }
        ~MPConflictBasedSearch(void);

        ConflictSet findConflict(
                const Map &map, const AgentSet &agentSet,
                const ConstraintsSet &constraints, const std::vector<int> &costs,
                std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
                const std::vector<std::list<Node>::iterator> &starts,
                const std::vector<std::list<Node>::iterator> &ends,
                int agentId = -1, bool findAllConflicts = false,
                bool withCardinalConflicts = false,
                const std::vector<MDD> &mdds = std::vector<MDD>()) override;

    private:
        void getOccupiedNodes(std::vector<SIPPNode> &nodes,
                              std::list<Node>::iterator it, std::list<Node>::iterator end,
                              int agentId, std::vector<int> &lastStart);

        void getConflicts(const Map &map, const AgentSet &agentSet,
                          const ConstraintsSet &constraints, const std::vector<int> &costs,
                          std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
                          const std::vector<std::list<Node>::iterator> &starts,
                          const std::vector<std::list<Node>::iterator> &ends,
                          std::map<std::pair<int, int>, std::set<IntervalBoundary>> &boundaries,
                          const std::vector<SIPPNode> &nodes, int curAgentId,
                          ConflictSet &conflictSet, bool getAllConflicts, bool withCardinalConflicts);

        void addBoundaries(std::map<std::pair<int, int>, std::set<IntervalBoundary>> &boundaries,
                           const std::vector<SIPPNode> &nodes, int curAgentId);

        Primitives *mp;
};

template<typename SearchType>
void MPConflictBasedSearch<SearchType>::getOccupiedNodes(std::vector<SIPPNode> &nodes,
                                                         std::list<Node>::iterator it,
                                                         std::list<Node>::iterator end,
                                                         int agentId, std::vector<int> &lastStart) {
    if (std::next(it) != end) {
        Primitive pr = this->mp->getPrimitive(std::next(it)->primitiveId);
        auto cells = pr.getCells();
        for (auto cell : cells) {
            SIPPNode node(it->i + cell.i, it->j + cell.j);
            int start = std::next(it)->g - pr.intDuration;
            node.startTime = start + cell.interval.first;
            node.endTime = start + cell.interval.second;
            nodes.push_back(node);
        }
    } else {
        SIPPNode node(it->i, it->j, nullptr, it->g);
        node.endTime = CN_INFINITY;
        nodes.push_back(node);
    }

    nodes[0].startTime = lastStart[agentId];
    if (nodes.size() > 1) {
        lastStart[agentId] = nodes.back().startTime;
        nodes.pop_back();
    }
}

template<typename SearchType>
void MPConflictBasedSearch<SearchType>::getConflicts(const Map &map, const AgentSet &agentSet,
                                                     const ConstraintsSet &constraints, const std::vector<int> &costs,
                                                     std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
                                                     const std::vector<std::list<Node>::iterator> &starts,
                                                     const std::vector<std::list<Node>::iterator> &ends,
                                                     std::map<std::pair<int, int>, std::set<IntervalBoundary>> &boundaries,
                                                     const std::vector<SIPPNode> &nodes, int curAgentId,
                                                     ConflictSet &conflictSet, bool getAllConflicts, bool withCardinalConflicts) {
    for (auto node : nodes) {
        auto pair = std::make_pair(node.i, node.j);
        auto cellBoundaries = boundaries.find(pair);
        if (cellBoundaries != boundaries.end()) {
            std::map<int, int> startTime;
            std::vector<Conflict> conflicts;
            auto it = cellBoundaries->second.begin();
            auto end = cellBoundaries->second.upper_bound(IntervalBoundary(node.endTime, 0, true));
            for (it; it != end; ++it) {
                if (it->end) {
                    if (it->time >= node.startTime) {
                        conflicts.push_back(Conflict(curAgentId, it->id, node, node, it->time, false));
                    }
                    if (startTime.find(it->id) != startTime.end()) {
                        startTime.erase(it->id);
                    }
                } else {
                    startTime[it->id] = it->time;
                }
            }
            for (auto pair : startTime) {
                conflicts.push_back(Conflict(curAgentId, pair.first, node, node, node.endTime, false));
            }

            for (auto conflict : conflicts) {
                if (withCardinalConflicts) {
                    std::vector<int> inc;
                    for (auto id : {conflict.id1, conflict.id2}) {
                        Constraint constraint(conflict.pos1.i, conflict.pos1.j, conflict.time, id);
                        ConstraintsSet agentConstraints = constraints.getAgentConstraints(id);
                        agentConstraints.addConstraint(constraint);
                        SearchResult searchResult = this->search->startSearch(map, agentSet, starts[id]->i, starts[id]->j,
                                                                              std::prev(ends[id])->i, std::prev(ends[id])->j, nullptr, true,
                                                                              true, 0, -1, -1, {}, agentConstraints);
                        if (!searchResult.pathfound) {
                            continue;
                        }
                        inc.push_back(std::max(searchResult.pathlength - costs[id], 0));
                        LLExpansions.push_back(searchResult.nodesexpanded);
                        LLNodes.push_back(searchResult.nodescreated);
                    }
                    for (; inc.size() < 2; inc.push_back(CN_INFINITY));
                    if (inc[0] > 0 && inc[1] > 0) {
                        conflict.minIncrease = std::min(inc[0], inc[1]);
                        conflictSet.addCardinalConflict(conflict);
                        if (!getAllConflicts) {
                            return;
                        }
                    } else if (inc[0] > 0 || inc[1] > 0) {
                        conflictSet.addSemiCardinalConflict(conflict);
                    } else {
                        conflictSet.addNonCardinalConflict(conflict);
                    }
                } else {
                    conflictSet.addNonCardinalConflict(conflict);
                    if (!getAllConflicts) {
                        return;
                    }
                }
            }
        }
    }
}

template<typename SearchType>
void MPConflictBasedSearch<SearchType>::addBoundaries(std::map<std::pair<int, int>, std::set<IntervalBoundary>> &boundaries,
                                                      const std::vector<SIPPNode> &nodes, int curAgentId) {
    for (auto node : nodes) {
        auto pair = std::make_pair(node.i, node.j);
        if (boundaries.find(pair) == boundaries.end()) {
            boundaries[pair] = {};
        }
        boundaries[pair].insert(IntervalBoundary(node.startTime, curAgentId, false));
        boundaries[pair].insert(IntervalBoundary(node.endTime, curAgentId, true));
    }
}

template<typename SearchType>
ConflictSet MPConflictBasedSearch<SearchType>::findConflict(
        const Map &map, const AgentSet &agentSet,
        const ConstraintsSet &constraints, const std::vector<int> &costs,
        std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
        const std::vector<std::list<Node>::iterator> &starts,
        const std::vector<std::list<Node>::iterator> &ends,
        int agentId, bool findAllConflicts,
        bool withCardinalConflicts, const std::vector<MDD> &mdds) {
    ConflictSet conflictSet;
    std::vector<std::list<Node>::iterator> iters = starts;
    std::set<int> notFinished;
    std::map<std::pair<int, int>, std::set<IntervalBoundary>> boundaries;
    std::vector<int> lastStart(starts.size(), 0);

    for (int i = 0; i < starts.size(); ++i) {
        if (i != agentId) {
            notFinished.insert(i);
        }
    }
    if (agentId != -1) {
        for (auto it = starts[agentId]; it != ends[agentId]; ++it) {
            std::vector<SIPPNode> nodes;
            getOccupiedNodes(nodes, it, ends[agentId], agentId, lastStart);
            addBoundaries(boundaries, nodes, agentId);
        }
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for (int i = 0; !notFinished.empty(); ++i) {
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > 30000) {
            return conflictSet;
        }

        std::set<int> newNotFinished;
        for (int j : notFinished) {
            std::vector<SIPPNode> nodes;
            if (std::next(iters[j]) != ends[j]) {
                newNotFinished.insert(j);
            }
            getOccupiedNodes(nodes, iters[j], ends[j], j, lastStart);
            getConflicts(map, agentSet, constraints, costs, LLExpansions, LLNodes, starts, ends,
                        boundaries, nodes, j, conflictSet, findAllConflicts, withCardinalConflicts);
            if (!findAllConflicts && !conflictSet.empty()) {
                return conflictSet;
            }
            if (agentId == -1) {
                addBoundaries(boundaries, nodes, j);
            }
            ++iters[j];
        }
        notFinished = newNotFinished;
    }
    return conflictSet;
}

#endif // MPCONFLICTBASEDSEARCH_H
