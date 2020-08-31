#ifndef TWOKNEIGHSIPP_H
#define TWOKNEIGHSIPP_H

#include "sipp.h"
#include "motion_primitives.h"
#include "two_k_neigh_sipp_node.h"

template <typename NodeType = TwoKNeighSIPPNode>
class TwoKNeighSIPP : public SIPP<NodeType>
{
public:
    void findSuccessors(std::list<NodeType> &successors,
                        const NodeType &curNode, const Map &map,
                        int goal_i, int goal_j, int agentId,
                        const ConstraintsSet &constraints,
                        bool withCAT, const ConflictAvoidanceTable &CAT) override;
    TwoKNeighSIPP(int k, int time_resolution, int scale, double agentSize);
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
    void getNeigboursWithoutChecks(const Map &map, const Node &cur, ISearch<> &search, std::list<Node> &successors) override;

    Primitives mp;

};

#endif // TWOKNEIGHSIPP_H
