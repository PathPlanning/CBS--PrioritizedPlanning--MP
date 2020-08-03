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
    TwoKNeighSIPP(int k);
    virtual ~TwoKNeighSIPP() {}

// private:
    void makePrimaryPath(Node &curNode, int endTime) override;
    void makeSecondaryPath(const Map &map) override {};

    Primitives mp;

};

#endif // TWOKNEIGHSIPP_H
