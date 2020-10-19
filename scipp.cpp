#include "scipp.h"

template<typename NodeType>
SCIPP<NodeType>::~SCIPP() {}

template<typename NodeType>
void SCIPP<NodeType>::splitBySoftConflicts(std::vector<std::pair<int, int>> &softConflictIntervals,
                                           int startTime, int endTime,
                                           const Primitive &pr,
                                           const ConflictAvoidanceTable &CAT) {
    boost::icl::interval_map<int, int> intervals;
    for (int k = 0; k < pr.cells.size(); ++k) {
        std::vector<std::pair<int, int>> cellSoftConflictIntervals;
        int cellStart = pr.cells[k].interval.first;
        CAT.getSoftConflictIntervals(cellSoftConflictIntervals, Node(pr.cells[k].i, pr.cells[k].j),
                                     startTime + cellStart,
                                     this->addWithInfCheck(endTime, cellStart),
                                     pr.cells[k].interval.second - pr.cells[k].interval.first + 1, k == 0);
        for (int i = 0; i < cellSoftConflictIntervals.size(); ++i) {
            if (cellSoftConflictIntervals[i].second > 0) {
                int intervalStart, intervalEnd;
                intervalStart = cellSoftConflictIntervals[i].first - cellStart;
                if (i == cellSoftConflictIntervals.size() - 1) {
                    intervalEnd = endTime;
                } else {
                    intervalEnd = cellSoftConflictIntervals[i + 1].first - cellStart;
                }
                boost::icl::interval<int>::type interval(intervalStart, intervalEnd + 1);
                intervals += std::make_pair(interval, cellSoftConflictIntervals[i].second);
            }
        }
    }

    int prevUpper = startTime;
    for (auto interval : intervals) {
        if (interval.first.lower() < prevUpper) {
            softConflictIntervals.push_back(std::make_pair(prevUpper, 0));
        }
        softConflictIntervals.push_back(std::make_pair(interval.first.lower(), interval.second));
        prevUpper = interval.first.upper();
    }
    if (prevUpper <= endTime) {
        softConflictIntervals.push_back(std::make_pair(prevUpper, 0));
    }
}

template<typename NodeType>
void SCIPP<NodeType>::addStartNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT) {
    this->updateEndTimeBySoftConflicts(node, CAT);
    this->open.insert(map, node, ISearch<NodeType>::withTime);
}

template class SCIPP<SCIPPNode>;
