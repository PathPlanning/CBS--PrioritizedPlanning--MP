#include "conflict_avoidance_table.h"
#include "motion_primitives.h"

void ConflictAvoidanceTable::getCells(std::vector<Cell> &cells,
                                      const std::list<Node>::const_iterator& start,
                                      const std::list<Node>::const_iterator& end,
                                      const Primitives *mp) {
    int lastNodeStart = 0;
    for (auto it = start; it != end; ++it) {
        if (it != start) {
            auto pr = mp->getPrimitive(it->primitiveId);
            pr.setSource(std::prev(it)->i, std::prev(it)->j);
            int startTime = it->g - pr.intDuration;
            pr.setStartTime(startTime);

            for (int i = 0; i < pr.cells.size() - 1; ++i) {
                if (i == 0) {
                    pr.cells[i].interval.first = lastNodeStart;
                }
                cells.push_back(pr.cells[i]);
            }

            if (std::next(it) == end) {
                if (pr.cells.size() == 1) {
                    pr.cells.back().interval.first = lastNodeStart;
                }
                pr.cells.back().interval.second = CN_INFINITY;
                cells.push_back(pr.cells.back());
            } else if (pr.cells.size() > 1) {
                lastNodeStart = pr.cells.back().interval.first + startTime;
            }
        }
    }
}

void ConflictAvoidanceTable::addCell(const Cell &cell) {
    auto pair = std::make_pair(cell.i, cell.j);
    if (intervals.find(pair) == intervals.end()) {
        intervals[pair] = boost::icl::interval_map<int, int>();
    }
    boost::icl::interval<int>::type interval(cell.interval.first, cell.interval.second + 1);
    intervals[pair] += std::make_pair(interval, 1);
}

void ConflictAvoidanceTable::addAgentPath(const std::list<Node>::const_iterator& start,
                                          const std::list<Node>::const_iterator& end,
                                          const Primitives *mp) {
    std::vector<Cell> cells;
    getCells(cells, start, end, mp);
    for (auto cell : cells) {
        addCell(cell);
    }
}

void ConflictAvoidanceTable::removeCell(const Cell &cell) {
    auto pair = std::make_pair(cell.i, cell.j);
    boost::icl::interval<int>::type interval(cell.interval.first, cell.interval.second + 1);
    intervals[pair] -= std::make_pair(interval, 1);
}

void ConflictAvoidanceTable::removeAgentPath(const std::list<Node>::const_iterator& start,
                                             const std::list<Node>::const_iterator& end,
                                             const Primitives *mp) {
    std::vector<Cell> cells;
    getCells(cells, start, end, mp);
    for (auto cell : cells) {
        removeCell(cell);
    }
}

int ConflictAvoidanceTable::getFirstSoftConflict(const Node & node, int startTime, int endTime) const {
    auto pair = std::make_pair(node.i, node.j);
    if (intervals.find(pair) != intervals.end()) {
        auto it = intervals.at(pair).find(boost::icl::interval<int>::type(startTime, endTime + 1));
        if (it != intervals.at(pair).end() && it->first.lower() <= endTime) {
            return std::max(startTime, it->first.lower());
        }
    }
    return -1;
}

/*int ConflictAvoidanceTable::getFutureConflictsCount(const Node & node, int time) const {
    int res = 0;
    auto it = nodeAgentsCount.upper_bound(std::make_tuple(node.i, node.j, time));
    for (; it != nodeAgentsCount.end() && std::get<0>(it->first) == node.i
                                   && std::get<1>(it->first) == node.j; ++it) {
        res += it->second;
    }
    return res;
}*/

void ConflictAvoidanceTable::getSoftConflictIntervals(std::vector<std::pair<int, int>> &res, const Node & node,
                                                      int startTime, int endTime, int duration, bool firstCell) const {
    auto pair = std::make_pair(node.i, node.j);
    if (intervals.find(pair) == intervals.end()) {
        res.push_back(std::make_pair(startTime, 0));
        return;
    }

    int start = startTime, count = 0;
    int prevUpper = startTime;
    auto it = intervals.at(pair).find(boost::icl::interval<int>::type(startTime, endTime + 1));
    auto end = intervals.at(pair).end();
    for (it; it != end; ++it) {
        boost::icl::interval<int>::type interval = it->first;
        if (interval.lower() > endTime + duration - 1) {
            break;
        }

        if (interval.lower() - prevUpper >= duration) {
            res.push_back(std::make_pair(prevUpper, 0));
            res.push_back(std::make_pair(interval.lower() - duration + 1, 1));
        }

        if (res.empty()) {
            res.push_back(std::make_pair(startTime, 1));
        }
        prevUpper = interval.upper();
    }
    if (prevUpper <= endTime) {
        res.push_back(std::make_pair(prevUpper, 0));
    }
}
