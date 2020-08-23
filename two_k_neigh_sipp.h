#ifndef TWOKNEIGHSIPP_H
#define TWOKNEIGHSIPP_H

#include "sipp.h"
#include "motion_primitives.h"
#include "two_k_neigh_sipp_node.h"

template <typename NodeType = TwoKNeighSIPPNode>
class TwoKNeighSIPP : public SIPP<NodeType>
{
public:
    std::list<NodeType> findSuccessors(const NodeType &curNode, const Map &map,
                                       int goal_i, int goal_j, int agentId,
                                       const std::unordered_set<Node, NodeHash> &occupiedNodes,
                                       const ConstraintsSet &constraints,
                                       bool withCAT, const ConflictAvoidanceTable &CAT) override;
    TwoKNeighSIPP(int k, int time_resolution);
    virtual ~TwoKNeighSIPP() {}
    Primitives *getMP() { return &mp; };

private:
    void getCellSafeIntervals(const Cell &cell,
                              int startTime, int endTime,
                              int agentId, const ConstraintsSet &constraints,
                              std::vector<std::pair<int, int>> &safeIntervals);
    int addWithInfCheck(const int lhs, const int rhs);
    void makePrimaryPath(Node &curNode, int endTime) override;
    void makeSecondaryPath(const Map &map) override {};

    Primitives mp;

};

#endif // TWOKNEIGHSIPP_H
