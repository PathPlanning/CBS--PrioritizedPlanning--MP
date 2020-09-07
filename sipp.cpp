#include "sipp.h"

template<typename NodeType>
void SIPP<NodeType>::setEndTime(NodeType& node, int start_i, int start_j, int startTime, int agentId, const ConstraintsSet &constraints) {
    node.endTime = constraints.getFirstConstraintTime(start_i, start_j, startTime, agentId);
    if (node.endTime < CN_INFINITY) {
        --node.endTime;
    }
}

template<typename NodeType>
void SIPP<NodeType>::updateEndTimeBySoftConflicts(NodeType &node, const ConflictAvoidanceTable &CAT) {
    int newEndTime = CAT.getFirstSoftConflict(node, node.startTime, node.endTime);
    if (newEndTime != -1) {
        node.endTime = newEndTime - 1;
    }
}

template<typename NodeType>
void SIPP<NodeType>::getCellSafeIntervals(const Cell &cell,
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
int SIPP<NodeType>::addWithInfCheck(const int lhs, const int rhs) {
    if (lhs == CN_INFINITY) {
        return lhs;
    }
    return lhs + rhs;
}

template<typename NodeType>
void SIPP<NodeType>::createSuccessorsFromNode(const NodeType &cur, NodeType &neigh, std::list<NodeType> &successors,
                                    int agentId, const ConstraintsSet &constraints,
                                    const ConflictAvoidanceTable &CAT, bool isGoal, Primitive &pr) {
    auto cells = pr.getCells();
    cells.back().interval.second = pr.intDuration;
    std::vector<std::pair<int, int>> safeIntervals;
    getCellSafeIntervals(cells.back(), cur.g, cur.endTime, agentId, constraints, safeIntervals);
    int newg = neigh.g;

    for (auto interval : safeIntervals) {
        int waitTime = std::max(interval.first, 0);
        neigh.startTime = interval.first + cur.g + cells.back().interval.first;
        neigh.endTime = addWithInfCheck(interval.second, cur.g + pr.intDuration);

        std::vector<IntervalBoundary> events;
        int safeCellsCount = 0;
        for (int i = 0; i < cells.size() - 1; ++i) {
            if (!constraints.hasConstraint(cells[i].i, cells[i].j, agentId)) {
                ++safeCellsCount;
                continue;
            }

            std::vector<std::pair<int, int>> cellSafeIntervals;
            getCellSafeIntervals(cells[i], cur.g + waitTime, addWithInfCheck(interval.second, cur.g),
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
                    neigh.startTime = interval.first + cur.g + cells.back().interval.first;
                    neigh.endTime = addWithInfCheck(interval.second, cur.g + pr.intDuration);
                    neigh.g = newg + std::max(beg, waitTime);
                    neigh.F = neigh.H + neigh.g;
                    successors.push_back(neigh);
                }
                --safeCellsCount;
            }
        }
    }
}

template<typename NodeType>
void SIPP<NodeType>::splitBySoftConflicts(std::vector<std::pair<int, int>> &softConflictIntervals,
                                          const NodeType & node, const NodeType & prevNode, std::pair<int, int> interval,
                                          const ConflictAvoidanceTable &CAT) {
    softConflictIntervals.push_back(std::make_pair(interval.first, 0));
}

template<typename NodeType>
bool SIPP<NodeType>::checkGoal(const NodeType &cur, int goalTime, int agentId, const ConstraintsSet &constraints) {
    return goalTime == -1 || cur.g <= goalTime;
}

template class SIPP<SIPPNode>;
template class SIPP<ZeroSCIPPNode>;
template class SIPP<SCIPPNode>;
