#ifndef ASTAR_H
#define ASTAR_H
#include "dijkstra.h"
#include "math.h"

//A* search.
template <typename NodeType = Node>
class Astar : public Dijkstra<NodeType>
{
    public:
        Astar(const Primitives &mp, bool withTime = true) : Dijkstra<NodeType>(mp, withTime), htype(CN_SP_MT_MANH) {}
        virtual ~Astar() {}
        double computeHFromCellToCell(int i1, int j1, int i2, int j2) override;
        void getPerfectHeuristic(const Map &map, const AgentSet &agentSet, Primitives &mp, int prevAgentCount = 0);

    protected:
        double euclideanDistance(int x1, int y1, int x2, int y2);
        double manhattanDistance(int x1, int y1, int x2, int y2);
        double chebyshevDistance(int x1, int y1, int x2, int y2);
        double diagonalDistance(int x1, int y1, int x2, int y2);
        double metric(int x1, int y1, int x2, int y2);
        virtual void getNeigboursWithoutChecks(const Map &map, const Node &cur, ISearch<> &search, std::list<Node> &successors);

        std::unordered_map<std::pair<Node, Node>, int, NodePairHash> perfectHeuristic;
        int htype;
};

#endif
