#ifndef TWO_K_NEIGH_SIPP_NODE_H
#define TWO_K_NEIGH_SIPP_NODE_H

#include "SIPP_node.h"

struct TwoKNeighSIPPNode : public SIPPNode
{
    //int primitiveId;

    TwoKNeighSIPPNode(int x = 0, int y = 0, Node *p = nullptr, int g_ = 0,
               double H_ = 0, int ConflictsCount = 0, int PrimitiveId = 0) :
        Node(x, y, p, g_, H_, ConflictsCount),
        SIPPNode(x, y, p, g_, H_, ConflictsCount) {
        primitiveId = PrimitiveId;
    }

    TwoKNeighSIPPNode(const Node &other) : Node(other) {}
};

#endif // TWO_K_NEIGH_SIPP_NODE_H
