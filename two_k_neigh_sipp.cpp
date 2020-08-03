#include "two_k_neigh_sipp.h"

template <typename NodeType>
TwoKNeighSIPP<NodeType>::TwoKNeighSIPP(int k) : SIPP<NodeType>() {
    this->htype = CN_SP_MT_EUCL;
    mp.makeTwoKNeigh(k);
}


template<typename NodeType>
std::list<NodeType> TwoKNeighSIPP<NodeType>::findSuccessors(const NodeType &curNode, const Map &map,
                                        int goal_i, int goal_j, int agentId,
                                        const std::unordered_set<Node, NodeHash> &occupiedNodes,
                                        const ConstraintsSet &constraints,
                                        bool withCAT, const ConflictAvoidanceTable &CAT)
{
    std::list<NodeType> successors;
    std::vector<Primitive> primitives = mp.getPrimitives(curNode.i, curNode.j, 0, 1, map);
    for (auto pr : primitives) {
        int newi = curNode.i + pr.target.i;
        int newj = curNode.j + pr.target.j;
        int newg = curNode.g + int(pr.duration * CN_PRECISION);
        double newh = this->computeHFromCellToCell(newi, newj, goal_i, goal_j) * CN_PRECISION;
        NodeType neigh(newi, newj, nullptr, newg, newh, 0, pr.id);

        std::vector<IntervalBoundary> events;
        std::set<int> safeCells;
        auto cells = pr.getCells();
        for (int i = 0; i < cells.size(); ++i) {
            int start = int(cells[i].interval.first * CN_PRECISION);
            int end = int(cells[i].interval.second * CN_PRECISION);
            if (i == cells.size() - 1) {
                end = int(pr.duration * CN_PRECISION);
            }
            int endTime = curNode.endTime;
            if (endTime != CN_INFINITY) {
                endTime += start;
            }
            std::vector<std::pair<int, int>> safeIntervals = constraints.getSafeIntervals(
                        curNode.i + cells[i].i, curNode.j + cells[i].j, agentId,
                        curNode.g + start, endTime, end - start + 1);

            for (auto interval : safeIntervals) {
                int endTime = interval.second;
                if (endTime != CN_INFINITY) {
                    endTime -= (start + curNode.g);
                }
                if (endTime >= 0) {
                    events.push_back(IntervalBoundary(interval.first - start - curNode.g, i, false));
                    events.push_back(IntervalBoundary(endTime, i, true));
                }
            }
        }

        std::sort(events.begin(), events.end());
        int beg;
        int i = 0;
        while (i < events.size()) {
            int curTime = events[i].time;
            for (i; i < events.size() && events[i].time == curTime && events[i].end == false; ++i) {
                safeCells.insert(events[i].id);
                if (safeCells.size() == cells.size()) {
                    beg = curTime;
                }
            }

            for (i; i < events.size() && events[i].time == curTime; ++i) {
                if (safeCells.size() == cells.size()) {
                    neigh.startTime += beg;
                    if (curTime == CN_INFINITY) {
                        neigh.endTime = CN_INFINITY;
                    } else {
                        neigh.endTime += curTime;
                    }
                    if (beg > 0) {
                        neigh.g += beg;
                        neigh.F += beg;
                    }
                    successors.push_back(neigh);
                }
                safeCells.erase(events[i].id);
            }
        }
    }
    return successors;
}

template<typename NodeType>
void TwoKNeighSIPP<NodeType>::makePrimaryPath(Node &curNode, int endTime)
{
    this->lppath.push_front(curNode);
    if (curNode.parent != nullptr) {
        makePrimaryPath(*(curNode.parent), curNode.g);
    }
}

template class TwoKNeighSIPP<TwoKNeighSIPPNode>;

