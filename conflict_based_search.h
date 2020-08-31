#ifndef CONFLICT_BASED_SEARCH_H
#define CONFLICT_BASED_SEARCH_H

#include "isearch.h"
#include "scipp.h"
#include "zero_scipp.h"
#include "two_k_neigh_sipp.h"
#include "cbs_node.h"
#include "multiagent_search_result.h"
#include "multiagent_search_inteface.h"
#include "config.h"
#include "conflict_avoidance_table.h"
#include "mdd.h"
#include "conflict_set.h"
#include <numeric>

template <typename SearchType = Astar<>>
class ConflictBasedSearch : public MultiagentSearchInterface
{
    public:
        ConflictBasedSearch();
        ConflictBasedSearch(SearchType* Search);
        ~ConflictBasedSearch(void);
        MultiagentSearchResult startSearch(const Map &map, const Config &config, AgentSet &agentSet) override;

        virtual ConflictSet findConflict(
                const Map &map, const AgentSet &agentSet,
                const ConstraintsSet &constraints, const std::vector<int> &costs,
                std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
                const std::vector<std::list<Node>::iterator> &starts,
                const std::vector<std::list<Node>::iterator> &ends,
                int agentId = -1, bool findAllConflicts = false,
                bool withCardinalConflicts = false,
                const std::vector<MDD> &mdds = std::vector<MDD>()) override;
    Primitives *mp;
protected:
        std::list<Node> getNewPath(const Map &map, const AgentSet &agentSet, const Agent &agent,
                                   const Constraint &constraint, const ConstraintsSet &constraints,
                                   const std::list<Node>::iterator pathStart,
                                   const std::list<Node>::iterator pathEnd,
                                   bool withCAT, const ConflictAvoidanceTable &CAT,
                                   std::vector<double> &lb, std::vector<int> &LLExpansions, std::vector<int> &LLNodes);
        CBSNode createNode(const Map &map, const AgentSet &agentSet, const Config &config,
                           const Conflict &conflict, const std::vector<int> &costs,
                           ConstraintsSet &constraints, int id1, int id2,
                           const Node &pos1, const Node &pos2,
                           std::vector<std::list<Node>::iterator> &starts,
                           std::vector<std::list<Node>::iterator> &ends,
                           ConflictAvoidanceTable &CAT, ConflictSet &conflictSet,
                           std::vector<MDD> &mdds, std::vector<double> &lb,
                           std::vector<int> &LLExpansions, std::vector<int> &LLNodes, CBSNode *parentPtr);


        SearchType*                     search;
};

template<typename SearchType>
ConflictSet ConflictBasedSearch<SearchType>::findConflict(
        const Map &map, const AgentSet &agentSet,
        const ConstraintsSet &constraints, const std::vector<int> &costs,
        std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
        const std::vector<std::list<Node>::iterator> &starts,
        const std::vector<std::list<Node>::iterator> &ends,
        int agentId, bool findAllConflicts,
        bool withCardinalConflicts, const std::vector<MDD> &mdds) {
    ConflictSet conflictSet;
    std::vector<std::list<Node>::iterator> iters = starts;
    std::vector<int> order;
    for (int i = 0; i < iters.size(); ++i) {
        if (i != agentId) {
            order.push_back(i);
        }
    }
    if (agentId != -1) {
        order.push_back(agentId);
    }
    for (int time = 0;; ++time) {
        std::unordered_multimap<Node, int, NodeHash> positions;
        std::unordered_multimap<std::pair<Node, Node>, int, NodePairHash> edges;
        std::vector<int> ids;

        int finished = 0;
        for (int i : order) {
            if (agentId == -1 || i == agentId) {
                auto posRange = positions.equal_range(*iters[i]);
                for (auto posIt = posRange.first; posIt != posRange.second; ++posIt) {
                    Conflict conflict(i, posIt->second, *iters[i], *iters[i], time, false);
                    int id1 = i, id2 = posIt->second;
                    int size1 = 0, size2 = 0;
                    if (withCardinalConflicts) {
                        size1 = mdds[id1].getLayerSize(time);
                        size2 = mdds[id2].getLayerSize(time);
                    }

                    if (size1 == 1 && size2 == 1) {
                        conflictSet.addCardinalConflict(conflict);
                        if (!findAllConflicts) {
                            return conflictSet;
                        }
                    } else if (size1 == 1 || size2 == 1) {
                        conflictSet.addSemiCardinalConflict(conflict);
                    } else {
                        conflictSet.addNonCardinalConflict(conflict);
                    }
                    if (!findAllConflicts && !withCardinalConflicts) {
                        return conflictSet;
                    }
                }
            }
            positions.insert(std::make_pair(*iters[i], i));

            if (std::next(iters[i]) != ends[i]) {
                if (agentId == -1 || i == agentId) {
                    auto edgeRange = edges.equal_range(std::make_pair(*std::next(iters[i]), *iters[i]));
                    for (auto edgeIt = edgeRange.first; edgeIt != edgeRange.second; ++edgeIt) {
                        Conflict conflict(i, edgeIt->second, *iters[i], *std::next(iters[i]), time + 1, true);
                        int id1 = i, id2 = edgeIt->second;
                        int inSize1 = 0, outSize1 = 0, inSize2 = 0, outSize2 = 0;
                        if (withCardinalConflicts) {
                            inSize1 = mdds[id1].getLayerSize(time);
                            outSize1 = mdds[id1].getLayerSize(time + 1);
                            inSize2 = mdds[id2].getLayerSize(time);
                            outSize2 = mdds[id2].getLayerSize(time + 1);
                        }

                        if ((inSize1 == 1 && outSize1 == 1) && (inSize1 == 2 && outSize2 == 1)) {
                            conflictSet.addCardinalConflict(conflict);
                            if (!findAllConflicts) {
                                return conflictSet;
                            }
                        } else if ((inSize1 == 1 && outSize1 == 1) || (inSize2 == 1 && outSize2 == 1)) {
                            conflictSet.addSemiCardinalConflict(conflict);
                        } else {
                            conflictSet.addNonCardinalConflict(conflict);
                        }
                        if (!findAllConflicts && !withCardinalConflicts) {
                            return conflictSet;
                        }
                    }
                }
                edges.insert(std::make_pair(std::make_pair(*iters[i], *std::next(iters[i])), i));
            }

            if (std::next(iters[i]) != ends[i]) {
                ++iters[i];
            } else {
                ++finished;
            }
        }

        if (finished == iters.size()) {
            break;
        }
    }
    return conflictSet;
}

#endif // CONFLICT_BASED_SEARCH_H
