#ifndef SIPP_H
#define SIPP_H

#include "astar.h"
#include "sipp_node.h"
#include "zero_scipp_node.h"

template <typename NodeType = SIPPNode>
class SIPP : virtual public Astar<NodeType>
{
public:
    SIPP(const Primitives &mp) : Astar<NodeType>(mp, true) {}
    virtual ~SIPP() {}

protected:
    void setEndTime(NodeType& node, int start_i, int start_j, int startTime, int agentId, const ConstraintsSet &constraints) override;
    int addWithInfCheck(const int lhs, const int rhs);
    void createSuccessorsFromNode(const NodeType &cur, NodeType &neigh, std::list<NodeType> &successors,
                                  int agentId, const ConstraintsSet &constraints,
                                  const ConflictAvoidanceTable &CAT, bool isGoal, Primitive &pr) override;
    bool checkGoal(const NodeType &cur, int goalTime, int agentId, const ConstraintsSet &constraints) override;
    void getCellSafeIntervals(const Cell &cell,
                              int startTime, int endTime,
                              int agentId, const ConstraintsSet &constraints,
                              std::vector<std::pair<int, int>> &safeIntervals);
    virtual void splitBySoftConflicts(std::vector<std::pair<int, int>> &softConflictIntervals,
                                      int startTime, int endTime,
                                      const Primitive &pr,
                                      const ConflictAvoidanceTable &CAT);
    virtual void setNeighG(const NodeType &cur, NodeType &neigh,
                           int agentId, const ConstraintsSet &constraints) {}
    virtual void addOptimalNode(const NodeType& cur, NodeType &neigh, std::pair<int, int> interval,
                                int agentId, const ConstraintsSet &constraints,
                                std::list<NodeType> &successors) {}
    int getNextConstraintTime(const NodeType& node, const ConstraintsSet &constraints, int agentId);
    virtual bool getOptimal(const NodeType &neigh) { return false; }
    virtual void setOptimal(NodeType &neigh, bool val) {}
    virtual bool checkSuboptimal(const NodeType &cur) { return true; }
    virtual bool canStay() override { return false; }
    virtual bool withZeroConflicts() { return false; }
    virtual void updateEndTimeBySoftConflicts(NodeType &node, const ConflictAvoidanceTable &CAT);
};

#endif // SIPP_H
