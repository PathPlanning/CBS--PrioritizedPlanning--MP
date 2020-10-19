#ifndef CONFLICTAVOIDANCETABLE_H
#define CONFLICTAVOIDANCETABLE_H

#include <unordered_map>
#include <list>
#include "node.h"
#include "map.h"
#include "search_queue.h"
#include "motion_primitives.h"
#include <boost/icl/interval_map.hpp>

class ConflictAvoidanceTable
{
public:
    void addCell(const Cell &cell);
    void addAgentPath(const std::list<Node>::const_iterator& start,
                      const std::list<Node>::const_iterator& end,
                      const Primitives *mp);
    void removeCell(const Cell &cell);
    void removeAgentPath(const std::list<Node>::const_iterator& start,
                         const std::list<Node>::const_iterator& end,
                         const Primitives *mp);
    int getFirstSoftConflict(const Node & node, int startTime, int endTime) const;
    void getSoftConflictIntervals(std::vector<std::pair<int, int>> &res, const Node & node,
                                  int startTime, int endTime, int duration, bool firstCell) const;
    void getCells(std::vector<Cell> &cells,
                  const std::list<Node>::const_iterator& start,
                  const std::list<Node>::const_iterator& end,
                  const Primitives *mp);

private:
    std::map<std::pair<int, int>, boost::icl::interval_map<int, int>> intervals;
};

#endif // CONFLICTAVOIDANCETABLE_H
