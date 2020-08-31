#include "two_k_neigh_sipp.h"

template <typename NodeType>
TwoKNeighSIPP<NodeType>::TwoKNeighSIPP(int k, int time_resolution, int scale, double agentSize) : SIPP<NodeType>() {
    this->withTime = true;
    if (k == 2) {
        this->htype = CN_SP_MT_MANH;
    } else {
        this->htype = CN_SP_MT_EUCL;
    }
    mp.makeTwoKNeigh(k, time_resolution, scale, agentSize);
}

template<typename NodeType>
void TwoKNeighSIPP<NodeType>::getCellSafeIntervals(const Cell &cell,
                                                   int startTime, int endTime,
                                                   int agentId, const ConstraintsSet &constraints,
                                                   std::vector<std::pair<int, int>> &safeIntervals) {
    int start = cell.interval.first;
    int end = cell.interval.second;
    if (endTime != CN_INFINITY) {
        endTime += start;
    }
    safeIntervals = constraints.getSafeIntervals(cell.i, cell.j, agentId, startTime + start,
                                                 endTime, end - start + 1);
    int lastNeg = -1;
    for (int i = 0; i < safeIntervals.size(); ++i) {
        safeIntervals[i].first -= (startTime + start);
        safeIntervals[i].second = addWithInfCheck(safeIntervals[i].second, -(startTime + start));
    }
}

template<typename NodeType>
int TwoKNeighSIPP<NodeType>::addWithInfCheck(const int lhs, const int rhs) {
    if (lhs == CN_INFINITY) {
        return lhs;
    }
    return lhs + rhs;
}

template<typename NodeType>
void TwoKNeighSIPP<NodeType>::getNeigboursWithoutChecks(const Map &map, const Node &cur,
                                                        ISearch<> &search, std::list<Node> &successors) {
    std::vector<Primitive> primitives;
    mp.getPrimitives(primitives, cur.i, cur.j, 0, 1, map);
    for (auto pr : primitives) {
        successors.push_back(Node(cur.i + pr.target.i, cur.j + pr.target.j, nullptr, cur.g + pr.intDuration));
    }
}

template<typename NodeType>
void TwoKNeighSIPP<NodeType>::findSuccessors(std::list<NodeType> &successors,
                                        const NodeType &curNode, const Map &map,
                                        int goal_i, int goal_j, int agentId,
                                        const ConstraintsSet &constraints,
                                        bool withCAT, const ConflictAvoidanceTable &CAT)
{
    std::vector<Primitive> primitives;
    mp.getPrimitives(primitives, curNode.i, curNode.j, 0, 1, map);
    for (auto pr : primitives) {
        pr.setSource(curNode.i, curNode.j);
        int newi = pr.target.i;
        int newj = pr.target.j;
        int newg = curNode.g + pr.intDuration;
        double newh = this->computeHFromCellToCell(newi, newj, goal_i, goal_j);
        NodeType neigh(newi, newj, nullptr, newg, newh, 0, pr.id);

        auto cells = pr.getCells();
        cells.back().interval.second = pr.intDuration;
        std::vector<std::pair<int, int>> safeIntervals;
        getCellSafeIntervals(cells.back(), curNode.g, curNode.endTime, agentId, constraints, safeIntervals);

        for (auto interval : safeIntervals) {
            int waitTime = std::max(interval.first, 0);
            neigh.startTime = interval.first + curNode.g + cells.back().interval.first;
            neigh.endTime = addWithInfCheck(interval.second, curNode.g + pr.intDuration);

            std::vector<IntervalBoundary> events;
            int safeCellsCount = 0;
            for (int i = 0; i < cells.size() - 1; ++i) {
                if (!constraints.hasConstraint(cells[i].i, cells[i].j, agentId)) {
                    ++safeCellsCount;
                    continue;
                }

                std::vector<std::pair<int, int>> cellSafeIntervals;
                getCellSafeIntervals(cells[i], curNode.g + waitTime, addWithInfCheck(interval.second, curNode.g),
                                     agentId, constraints, cellSafeIntervals);

                for (auto cellInterval : cellSafeIntervals) {
                    if (i > 0 || cellInterval.first + waitTime <= 0) {
                        events.push_back(IntervalBoundary(cellInterval.first + waitTime, i, false));
                        events.push_back(IntervalBoundary(addWithInfCheck(cellInterval.second, waitTime), i, true));
                    }
                }
            }

            if (events.empty() && safeCellsCount == cells.size() - 1) {
                neigh.g = newg + waitTime;
                neigh.F = neigh.H + neigh.g;
                successors.push_back(neigh);
                continue;
            }

            std::sort(events.begin(), events.end());
            int beg;
            int i = 0;
            while (i < events.size()) {
                int curTime = events[i].time;
                for (i; i < events.size() && events[i].time == curTime && events[i].end == false; ++i) {
                    ++safeCellsCount;
                    if (safeCellsCount == cells.size() - 1) {
                        beg = curTime;
                    }
                }

                for (i; i < events.size() && events[i].time == curTime; ++i) {
                    if (safeCellsCount == cells.size() - 1) {
                        neigh.startTime = interval.first + curNode.g + cells.back().interval.first;
                        neigh.endTime = addWithInfCheck(interval.second, curNode.g + pr.intDuration);
                        neigh.g = newg + std::max(beg, waitTime);
                        neigh.F = neigh.H + neigh.g;
                        successors.push_back(neigh);
                    }
                    --safeCellsCount;
                }
            }
        }
    }
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

