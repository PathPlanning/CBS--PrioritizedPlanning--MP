#ifndef ZEROSCIPP_H
#define ZEROSCIPP_H

#include "sipp.h"
#include "zero_scipp_node.h"

template <typename NodeType = ZeroSCIPPNode>
class ZeroSCIPP : public SIPP<NodeType>
{
public:
    ZeroSCIPP(const Primitives &mp, double Weight = 1.0, bool GenSuboptFromOpt = false) :
        Astar<NodeType>(mp, true),
        SIPP<NodeType>(mp), weight(Weight), genSuboptFromOpt(GenSuboptFromOpt)  {}
    virtual ~ZeroSCIPP();

protected:
    void splitBySoftConflicts(std::vector<std::pair<int, int>> &softConflictIntervals,
                              const NodeType & node, const NodeType & prevNode, std::pair<int, int> interval,
                              const ConflictAvoidanceTable &CAT) override;
    bool binarySplitting() { return true; }
    bool getOptimal(const NodeType &neigh) override;
    void setOptimal(NodeType &neigh, bool val) override;
    bool checkSuboptimal(const NodeType &cur) override;
    void addOptimalNode(const NodeType& cur, NodeType &neigh, std::pair<int, int> interval,
                        int agentId, const ConstraintsSet &constraints,
                        std::list<NodeType> &successors) override;
    bool withZeroConflicts() override { return true; };
    void addStartNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT) override;
    void addSuboptimalNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT) override;

    double weight;
    bool genSuboptFromOpt;
};


#endif // ZEROSCIPP_H
