#include "astar.h"

template<typename NodeType>
double Astar<NodeType>::computeHFromCellToCell(int i1, int j1, int i2, int j2)
{
    auto it = this->perfectHeuristic.find(std::make_pair(NodeType(i1, j1), NodeType(i2, j2)));
    if (it != this->perfectHeuristic.end()) {
        return it->second;
    }
    return metric(i1, j1, i2, j2);
}

template<typename NodeType>
double Astar<NodeType>::manhattanDistance(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

template<typename NodeType>
double Astar<NodeType>::euclideanDistance(int x1, int y1, int x2, int y2) {
    return sqrt(double((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)));
}

template<typename NodeType>
double Astar<NodeType>::metric(int x1, int y1, int x2, int y2) {
    if (htype == CN_SP_MT_MANH) {
        return manhattanDistance(x1, y1, x2, y2);
    } else {
        return euclideanDistance(x1, y1, x2, y2);
    }
}

template<typename NodeType>
void Astar<NodeType>::getNeigboursWithoutChecks(const Map &map, const Node &cur,
                                                ISearch<> &search, std::list<Node> &successors) {
    search.findSuccessors(successors, cur, map);
}

template<typename NodeType>
void Astar<NodeType>::getPerfectHeuristic(const Map &map, const AgentSet &agentSet, Primitives &mp, int prevAgentCount) {
    ISearch<> search(mp, false);
    for (int i = prevAgentCount; i < agentSet.getAgentCount(); ++i) {
        SearchQueue<Node> queue;
        std::unordered_set<int> visited;
        Node goal = Node(agentSet.getAgent(i).getGoal_i(), agentSet.getAgent(i).getGoal_j());
        queue.insert(map, goal, false);
        while (!queue.empty()) {
            Node cur = queue.getFront();
            queue.erase(map, cur, false);
            visited.insert(cur.convolution(map.getMapWidth(), map.getMapHeight()));
            perfectHeuristic[std::make_pair(cur, goal)] = cur.g;
            std::list<Node> successors;
            getNeigboursWithoutChecks(map, cur, search, successors);
            for (auto neigh : successors) {
                if (visited.find(neigh.convolution(map.getMapWidth(), map.getMapHeight())) == visited.end()) {
                    queue.insert(map, neigh, false);
                }
            }
        }
    }
}

template class Astar<Node>;
template class Astar<SIPPNode>;
template class Astar<ZeroSCIPPNode>;
template class Astar<SCIPPNode>;
template class Astar<FSNode>;
