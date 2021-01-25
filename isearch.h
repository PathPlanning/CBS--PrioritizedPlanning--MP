#ifndef ISEARCH_H
#define ISEARCH_H
#include "ilogger.h"
#include "searchresult.h"
#include "constraint.h"
#include "constraints_set.h"
#include "conflict_avoidance_table.h"
#include "search_queue.h"
#include "zero_scipp_node.h"
#include "motion_primitives.h"
#include <list>
#include <vector>
#include <math.h>
#include <limits>
#include <chrono>
#include <set>
#include <map>
#include <algorithm>
#include <unordered_set>
#include <queue>

template <typename NodeType = Node>
class ISearch
{
    public:
        ISearch(const Primitives &mp, bool WithTime = false);
        virtual ~ISearch(void);
        SearchResult startSearch(const Map &map, const AgentSet &agentSet,
                                 int start_i, int start_j, int goal_i = 0, int goal_j = 0,
                                 int startAngleId = 0, int goalAngleId = -1,
                                 int startTime = 0, int goalTime = -1, int maxTime = -1,
                                 const ConstraintsSet &constraints = ConstraintsSet(),
                                 bool withCAT = false, const ConflictAvoidanceTable &CAT = ConflictAvoidanceTable());

        virtual void findSuccessors(std::list<NodeType> &successors,
                                                   const NodeType &curNode, const Map &map, int goal_i = 0, int goal_j = 0, int agentId = -1,
                                                   const ConstraintsSet &constraints = ConstraintsSet(),
                                                   bool withCAT = false, const ConflictAvoidanceTable &CAT = ConflictAvoidanceTable());

        std::unordered_map<std::pair<NodeType, NodeType>, int, NodePairHash> getPerfectHeuristic(const Map &map, const AgentSet &agentSet);
        Primitives *getMP() { return &this->mp; };
        // void getPerfectHeuristic(const Map &map, const AgentSet &agentSet);
        virtual double computeHFromCellToCell(int start_i, int start_j, int fin_i, int fin_j) {return 0;}

        Primitives mp;
        static long long T;

    protected:
        //CODE HERE
        //Try to split class functionality to the methods that can be re-used in successors classes,
        //e.g. classes that realize A*, JPS, Theta* algorithms

        //Hint 1. You definetely need class variables for OPEN and CLOSE

        //Hint 2. It's a good idea to define a heuristic calculation function, that will simply return 0
        //for non-heuristic search methods like Dijkstra

        //Hint 3. It's a good idea to define function that given a node (and other stuff needed)
        //will return it's sucessors, e.g. unordered list of nodes

        //Hint 4. It's a good idea to define a resetParent function that will be extensively used for Theta*
        //and for A*/Dijkstra/JPS will exhibit "default" behaviour

        //Hint 5. The last but not the least: working with OPEN and CLOSE is the core
        //so think of the data structures that needed to be used, about the wrap-up classes (if needed)
        //Start with very simple (and ineffective) structures like list or vector and make it work first
        //and only then begin enhancement

        virtual void makePrimaryPath(Node &curNode, int endTime);//Makes path using back pointers
        virtual void makeSecondaryPath(const Map &map);//Makes another type of path(sections or points)
        virtual void setEndTime(NodeType& node, int start_i, int start_j, int startTime, int agentId, const ConstraintsSet &constraints);
        virtual int getNextConstraintTime(const NodeType& node, const ConstraintsSet &constraints, int agentId);
        virtual void setHC(NodeType &neigh, const NodeType &cur,
                           const ConflictAvoidanceTable &CAT, bool isGoal) {}
        virtual void createSuccessorsFromNode(const NodeType &cur, NodeType &neigh, std::list<NodeType> &successors,
                                              int agentId, const ConstraintsSet &constraints,
                                              const ConflictAvoidanceTable &CAT, bool isGoal, Primitive &pr);
        virtual bool checkGoal(const NodeType &cur, int goalTime, int agentId, const ConstraintsSet &constraints);
        virtual void addStartNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT);
        virtual void addSuboptimalNode(NodeType &node, const Map &map, const ConflictAvoidanceTable &CAT) {}
        virtual bool checkOpenEmpty();
        virtual bool canStay() { return withTime; }
        virtual int getFocalSize() { return 0; }
        virtual NodeType getCur(const Map& map);
        virtual void subtractFutureConflicts(NodeType &node) {}
        virtual bool updateFocal(const NodeType& neigh, const Map& map);
        virtual double getMinFocalF();
        virtual void clearLists();

        SearchResult                        sresult;
        std::list<Node>                     lppath, hppath;
        SearchQueue<NodeType>               open;
        std::unordered_map<int, NodeType>   close;
        bool                                withTime;
        //need to define open, close;

};


#endif
