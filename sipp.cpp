#include "sipp.h"

template<typename NodeType>
int SIPP<NodeType>::getNextConstraintTime(const NodeType& node, const ConstraintsSet &constraints, int agentId) {
    return node.endTime;
}


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
    //std::vector<std::pair<int, int>> safeIntervals;
    //getCellSafeIntervals(cells.back(), cur.g, cur.endTime, agentId, constraints, safeIntervals);

    std::vector<std::pair<int, int>> safeIntervals = constraints.getSafeIntervals(
                neigh.i, neigh.j, agentId,
                cur.g + pr.intDuration,
                addWithInfCheck(cur.endTime, pr.intDuration));

    for (auto interval : safeIntervals) {
        neigh.startTime = interval.first;
        neigh.endTime = interval.second;
        int waitTime = std::max(0, neigh.startTime - cur.g - pr.intDuration);
        int endTime = addWithInfCheck(neigh.endTime, -pr.intDuration);

        while (waitTime != CN_INFINITY) {
            int oldWaitTime = waitTime;
            for (int i = 0; i < cells.size(); ++i) {
                waitTime = constraints.getNewWaitTime(
                            cells[i], cur.g, waitTime, endTime, agentId);
                if (waitTime == CN_INFINITY) {
                    break;
                }
            }
            if (oldWaitTime == waitTime) {
                break;
            }
        }
        if (waitTime == CN_INFINITY) {
            continue;
        }

        if (constraints.getFirstConstraintTime(cells[0].i, cells[0].j, cur.g, agentId) <= cur.g + waitTime) {
            break;
        }

        int startTime = cur.g + waitTime;
        for (auto cell : cells) {
            int constraintTime = constraints.getFirstConstraintTime(cell.i, cell.j,
                                                                    startTime + cell.interval.first,
                                                                    agentId);
            if (constraintTime != CN_INFINITY) {
                endTime = std::min(endTime, constraintTime - cell.interval.second - 1);
            }
        }

        std::vector<std::pair<int, int>> softConflictIntervals;
        splitBySoftConflicts(softConflictIntervals, startTime, endTime, pr, CAT);
        for (auto softConflictInterval : softConflictIntervals) {
            neigh.g = softConflictInterval.first + pr.intDuration;
            neigh.F = neigh.g + neigh.H;
            neigh.conflictsCount = softConflictInterval.second;
            this->setHC(neigh, cur, CAT, false);
            successors.push_back(neigh);
        }
    }
}

template<typename NodeType>
void SIPP<NodeType>::splitBySoftConflicts(std::vector<std::pair<int, int>> &softConflictIntervals,
                                          int startTime, int endTime,
                                          const Primitive &pr,
                                          const ConflictAvoidanceTable &CAT) {
    softConflictIntervals.push_back(std::make_pair(startTime, 0));
}

template<typename NodeType>
bool SIPP<NodeType>::checkGoal(const NodeType &cur, int goalTime, int agentId, const ConstraintsSet &constraints) {
    return goalTime == -1 || cur.g <= goalTime;
}

template class SIPP<SIPPNode>;
template class SIPP<ZeroSCIPPNode>;
template class SIPP<SCIPPNode>;
