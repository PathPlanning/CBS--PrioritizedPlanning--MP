#ifndef SCIPP_NODE_H
#define SCIPP_NODE_H

#include "sipp_node.h"
#include "fs_node.h"

struct SCIPPNode : public SIPPNode, FSNode
{
    SCIPPNode(int x = 0, int y = 0, Node *p = nullptr, int g_ = 0, double H_ = 0, int ConflictsCount = 0, int PrimitiveId = -1, int AngleType = 0, int Speed = 0, int hc_ = 0) :
        Node(x, y, p, g_, H_, ConflictsCount, PrimitiveId, AngleType, Speed),
        SIPPNode(x, y, p, g_, H_, ConflictsCount, PrimitiveId, AngleType, Speed),
        FSNode(x, y, p, g_, H_, ConflictsCount, PrimitiveId, AngleType, Speed, hc_)
    {}

    SCIPPNode(const Node &other) : Node(other) {}

};

#endif
