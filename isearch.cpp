#include "isearch.h"

template<typename NodeType>
ISearch<NodeType>::ISearch(const Primitives &MP, bool WithTime)
{
    mp = MP;
    withTime = WithTime;
}

template<typename NodeType>
ISearch<NodeType>::~ISearch(void) {}

template<typename NodeType>
long long ISearch<NodeType>::T = 0;

template<typename NodeType>
SearchResult ISearch<NodeType>::startSearch(const Map &map, const AgentSet &agentSet,
                                  int start_i, int start_j, int goal_i, int goal_j,
                                  int startAngleId, int goalAngleId, int startTime, int goalTime, int maxTime,
                                  const ConstraintsSet &constraints,
                                  bool withCAT, const ConflictAvoidanceTable &CAT)
{
    sresult.pathfound = false;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    if (goalTime != -1) {
        maxTime = goalTime;
    }
    NodeType cur;
    int agentId = -1;
    if (agentSet.isOccupied(start_i, start_j)) {
        agentId = agentSet.getAgentId(start_i, start_j);
    }

    if (withCAT) {
        open = SearchQueue<NodeType>([](const NodeType &lhs, const NodeType &rhs) {
            return std::tuple<int, int, int, int, int>(lhs.F, lhs.conflictsCount, -lhs.g, lhs.i, lhs.j) <
                    std::tuple<int, int, int, int, int>(rhs.F, rhs.conflictsCount, -rhs.g, rhs.i, rhs.j);
        });
    }

    clearLists();
    sresult.numberofsteps = 0;
    cur = NodeType(start_i, start_j, nullptr, startTime,
                   computeHFromCellToCell(start_i, start_j, goal_i, goal_j));
    setEndTime(cur, start_i, start_j, startTime, agentId, constraints);
    if (mp.withTurns()) {
        cur.angleId = startAngleId;
    }
    addStartNode(cur, map, CAT);
    addSuboptimalNode(cur, map, CAT);

    while(!checkOpenEmpty()) {
        ++sresult.numberofsteps;
        if (sresult.numberofsteps % 100000 == 0) {
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            if (elapsedMilliseconds > 10000) {
                break;
            }
        }

        cur = getCur(map);
        close[cur.convolution(map.getMapWidth(), map.getMapHeight(), withTime)] = cur;

        NodeType *curPtr = &(close.find(cur.convolution(map.getMapWidth(), map.getMapHeight(), withTime))->second);
        if (cur.i == goal_i && cur.j == goal_j && (goalAngleId == -1 || !mp.withTurns() || cur.angleId == goalAngleId)) {
            bool hasFutureConstraint = false;
            for (auto cell : mp.covering) {
                if (constraints.hasFutureConstraint(cur.i + cell.i, cur.j + cell.j, cur.g, agentId)) {
                    hasFutureConstraint = true;
                    break;
                }
            }
            if (!hasFutureConstraint && checkGoal(cur, goalTime, agentId, constraints)) {
                sresult.pathfound = true;
                break;
            } else {
                subtractFutureConflicts(cur);
            }
        }

        if (maxTime == -1 || cur.g < maxTime) {
            std::list<NodeType> successors;
            findSuccessors(successors, cur, map, goal_i, goal_j, agentId, constraints, withCAT, CAT);
            for (auto neigh : successors) {
                if (close.find(neigh.convolution(map.getMapWidth(), map.getMapHeight(), withTime)) == close.end()) {
                    neigh.parent = curPtr;
                    if (!updateFocal(neigh, map)) {
                        open.insert(map, neigh, withTime);
                    }
                }
            }
        }
    }

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();

    sresult.time = static_cast<double>(elapsedMilliseconds) / 1000;
    sresult.nodescreated = open.size() + close.size() + getFocalSize();
    sresult.nodesexpanded = close.size();

    //if (withTime) {
    //    std::cout << sresult.numberofsteps << std::endl;
    //}

    //std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " " << sresult.nodesexpanded << std::endl;

    if (sresult.pathfound) {
        sresult.pathlength = cur.g;
        sresult.minF = std::min(double(cur.F), getMinFocalF());
        sresult.lastNode = cur;
        lppath.clear();
        hppath.clear();
        makePrimaryPath(cur, goalTime == -1 ? -1 : goalTime + 1);
        makeSecondaryPath(map);
        sresult.hppath = &hppath; //Here is a constant pointer
        sresult.lppath = &lppath;
    }
    return sresult;
}

template<typename NodeType>
void ISearch<NodeType>::findSuccessors(std::list<NodeType> &successors,
                                        const NodeType &curNode, const Map &map,
                                        int goal_i, int goal_j, int agentId,
                                        const ConstraintsSet &constraints,
                                        bool withCAT, const ConflictAvoidanceTable &CAT)
{
    std::vector<Primitive> primitives, turns;
    mp.getPrimitives(primitives, curNode.i, curNode.j, curNode.angleId, curNode.speed, map);
    if (curNode.speed == 0) {
        mp.getTurns(turns, curNode.angleId);
        primitives.insert(primitives.end(), turns.begin(), turns.end());
    }
    for (auto pr : primitives) {
        pr.setSource(curNode.i, curNode.j);
        int newi = pr.target.i;
        int newj = pr.target.j;
        int newg = curNode.g + pr.intDuration;
        double newh = this->computeHFromCellToCell(newi, newj, goal_i, goal_j);
        NodeType neigh(newi, newj, nullptr, newg, newh, 0, pr.id, pr.target.angle_id, pr.target.speed);
        createSuccessorsFromNode(curNode, neigh, successors, agentId, constraints, CAT,
                                 neigh.i == goal_i && neigh.j == goal_j, pr);
    }
}

template<typename NodeType>
void ISearch<NodeType>::clearLists() {
    open.clear();
    close.clear();
}

template<typename NodeType>
void ISearch<NodeType>::addStartNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT) {
    open.insert(map, node, withTime);
}

template<typename NodeType>
bool ISearch<NodeType>::checkOpenEmpty() {
    return open.empty();
}

template<typename NodeType>
NodeType ISearch<NodeType>::getCur(const Map& map) {
    NodeType cur = open.getFront();
    open.erase(map, cur, withTime);
    return cur;
}

template<typename NodeType>
bool ISearch<NodeType>::updateFocal(const NodeType& neigh, const Map& map) {
    return false;
}

template<typename NodeType>
double ISearch<NodeType>::getMinFocalF() {
    return CN_INFINITY;
}

template<typename NodeType>
void ISearch<NodeType>::setEndTime(NodeType& node, int start_i, int start_j, int startTime, int agentId, const ConstraintsSet &constraints) {
    return;
}

template<typename NodeType>
int ISearch<NodeType>::getNextConstraintTime(const NodeType& node, const ConstraintsSet &constraints, int agentId) {
    return constraints.getFirstConstraintTime(node.i, node.j, node.g, agentId);
}


template<typename NodeType>
bool ISearch<NodeType>::checkGoal(const NodeType &cur, int goalTime, int agentId, const ConstraintsSet &constraints) {
    return goalTime == -1 || cur.g == goalTime;
}

template<typename NodeType>
void ISearch<NodeType>::createSuccessorsFromNode(const NodeType &cur, NodeType &neigh, std::list<NodeType> &successors,
                                       int agentId, const ConstraintsSet &constraints,
                                       const ConflictAvoidanceTable &CAT, bool isGoal, Primitive &pr) {
    for (auto cell : pr.cells) {
        if (constraints.hasNodeConstraint(cell.i, cell.j, cur.g + cell.interval.first,
                                          agentId, cell.interval.second - cell.interval.first + 1)) {
            return;
        }
    }
    setHC(neigh, cur, CAT, isGoal);
    successors.push_back(neigh);
}

template<typename NodeType>
void ISearch<NodeType>::makePrimaryPath(Node &curNode, int endTime)
{
    this->lppath.push_front(curNode);
    if (curNode.parent != nullptr) {
        makePrimaryPath(*(curNode.parent), curNode.g);
    }
}

template<typename NodeType>
void ISearch<NodeType>::makeSecondaryPath(const Map &map)
{
    auto it = lppath.begin();
    hppath.push_back(*it);
    ++it;
    for (it; it != lppath.end(); ++it) {
        auto prevIt = it;
        --prevIt;
        auto nextIt = it;
        ++nextIt;
        if (nextIt == lppath.end() ||
            (it->i - prevIt->i) * (nextIt->j - it->j) != (it->j - prevIt->j) * (nextIt->i - it->i)) {
            hppath.push_back(*it);
        }
    }
}

template class ISearch<Node>;
template class ISearch<SIPPNode>;
template class ISearch<ZeroSCIPPNode>;
template class ISearch<SCIPPNode>;
template class ISearch<FSNode>;
