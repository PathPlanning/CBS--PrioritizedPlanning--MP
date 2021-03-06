#ifndef SCIPP_H
#define SCIPP_H

#include "focalsearch.h"
#include "sipp.h"
#include "scipp_node.h"

template <typename NodeType = SCIPPNode>
class SCIPP : public SIPP<NodeType>, FocalSearch<NodeType>
{
public:
    SCIPP(const Primitives &mp, double FocalW = 1.0) :
        Astar<NodeType>(mp, true), SIPP<NodeType>(mp), FocalSearch<NodeType>(mp, true, FocalW) {}
    virtual ~SCIPP();

protected:
    void splitBySoftConflicts(std::vector<std::pair<int, int>> &softConflictIntervals,
                              int startTime, int endTime,
                              const Primitive &pr,
                              const ConflictAvoidanceTable &CAT) override;
    bool canStay() override { return true; }
    void addStartNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT) override;
};

#endif // SCIPP_H
