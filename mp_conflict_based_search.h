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
            this->mp = Search->mp;
        }
        ~MPConflictBasedSearch(void);

        ConflictSet findConflict(const std::vector<std::list<Node>::iterator> &starts,
                                 const std::vector<std::list<Node>::iterator> &ends,
                                 int agentId = -1, bool findAllConflicts = false,
                                 bool withCardinalConflicts = false,
                                 const std::vector<MDD> &mdds = std::vector<MDD>()) override;

    private:
        Primitives mp;
};

template<typename SearchType>
ConflictSet MPConflictBasedSearch<SearchType>::findConflict(const std::vector<std::list<Node>::iterator> &starts,
                                                            const std::vector<std::list<Node>::iterator> &ends,
                                                            int agentId, bool findAllConflicts,
                                                            bool withCardinalConflicts, const std::vector<MDD> &mdds) {
    ConflictSet conflictSet;
    std::vector<std::list<Node>::iterator> iters = starts;
    std::set<int> notFinished;
    std::map<std::pair<int, int>, std::set<IntervalBoundary>> boundaries;
    for (int i = 0; i < starts.size(); ++i) {
        notFinished.insert(i);
    }
    for (int i = 0; !notFinished.empty(); ++i) {
        std::set<int> newNotFinished;
        for (int j : notFinished) {
            // std::cout << j << " " << iters[j]->i << " " << iters[j]->j << std::endl;
            std::vector<SIPPNode> nodes;
            if (std::next(iters[j]) != ends[j]) {
                Primitive pr = this->mp.getPrimitive(std::next(iters[j])->primitiveId);
                auto cells = pr.getCells();
                for (auto cell : cells) {
                    SIPPNode node(iters[j]->i + cell.i, iters[j]->j + cell.j);
                    int start = std::next(iters[j])->g - int(pr.duration * CN_PRECISION);
                    node.startTime = start + int(cell.interval.first * CN_PRECISION);
                    node.endTime = start + int(cell.interval.second * CN_PRECISION);
                    nodes.push_back(node);
                }
                newNotFinished.insert(j);
            } else {
                SIPPNode node(iters[j]->i, iters[j]->j, nullptr, iters[j]->g);
                node.endTime = CN_INFINITY;
                nodes.push_back(node);
            }

            for (auto node : nodes) {
                auto pair = std::make_pair(node.i, node.j);
                auto cellBoundaries = boundaries.find(pair);
                if (cellBoundaries != boundaries.end()) {
                    auto it = cellBoundaries->second.lower_bound(IntervalBoundary(node.startTime));
                    if (it != cellBoundaries->second.end() && it->id == j) {
                        ++it;
                    }
                    if (it != cellBoundaries->second.end() && it->time <= node.endTime) {
                        Conflict conflict(j, it->id, node, node, it->time, false);
                        if (!it->end) {
                            int id = it->id;
                            for (it = std::next(it); it->time <= node.endTime && it->id != id; ++it) {}
                            conflict.time = std::min(node.endTime, it->time);
                        }
                        conflictSet.addNonCardinalConflict(conflict);
                        return conflictSet;
                    }
                } else {
                    boundaries[pair] = {};
                }
                boundaries[pair].insert(IntervalBoundary(node.startTime, j, false));
                boundaries[pair].insert(IntervalBoundary(node.endTime, j, true));
            }
            ++iters[j];
        }
        notFinished = newNotFinished;
    }
}

#endif // MPCONFLICTBASEDSEARCH_H
