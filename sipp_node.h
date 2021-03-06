#ifndef SIPP_NODE_H
#define SIPP_NODE_H

#include "node.h"

struct SIPPNode : virtual public Node
{
    int     startTime;
    int     endTime;

    SIPPNode(int x = 0, int y = 0, Node *p = nullptr, int g_ = 0, double H_ = 0,
             int ConflictsCount = 0, int PrimitiveId = -1, int AngleType = 0, int Speed = 0, int hc_ = 0) :
        Node(x, y, p, g_, H_, ConflictsCount, PrimitiveId, AngleType, Speed), startTime(g_), endTime(g_) {}

    SIPPNode(const Node &other) : Node(other) {}

    virtual int convolution(int width, int height, bool withTime = true) const {
        int res = withTime ? width * height * startTime : 0;
        return 2 * 8 * (res + i * width + j) + 2 * angleId + speed;
    }

    bool operator< (const SIPPNode &other) const {
        return std::tuple<int, int, int, int, int, int, int>(F, -g, -startTime, j, i, speed, angleId) <
                std::tuple<int, int, int, int, int, int, int>(other.F, -other.g, -other.startTime,
                                                              other.j, other.i, other.speed, other.angleId);
    }

};

#endif
