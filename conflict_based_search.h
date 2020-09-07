#ifndef CONFLICT_BASED_SEARCH_H
#define CONFLICT_BASED_SEARCH_H

#include "isearch.h"
#include "scipp.h"
#include "zero_scipp.h"
#include "cbs_node.h"
#include "multiagent_search_result.h"
#include "multiagent_search_inteface.h"
#include "config.h"
#include "conflict_avoidance_table.h"
#include "conflict_set.h"
#include <numeric>

template <typename SearchType = Astar<>>
class ConflictBasedSearch : public MultiagentSearchInterface
{
    public:
        ConflictBasedSearch();
        ConflictBasedSearch(SearchType* Search);
        ~ConflictBasedSearch(void);
        MultiagentSearchResult startSearch(const Map &map, const Config &config, AgentSet &agentSet) override;

        virtual ConflictSet findConflict(
                const Map &map, const AgentSet &agentSet,
                const ConstraintsSet &constraints, const std::vector<int> &costs,
                std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
                const std::vector<std::list<Node>::iterator> &starts,
                const std::vector<std::list<Node>::iterator> &ends,
                int agentId = -1, bool findAllConflicts = false,
                bool withCardinalConflicts = false) override;
    Primitives *mp;
protected:
        std::list<Node> getNewPath(const Map &map, const AgentSet &agentSet, const Agent &agent,
                                   const Constraint &constraint, const ConstraintsSet &constraints,
                                   const std::list<Node>::iterator pathStart,
                                   const std::list<Node>::iterator pathEnd,
                                   bool withCAT, const ConflictAvoidanceTable &CAT,
                                   std::vector<double> &lb, std::vector<int> &LLExpansions, std::vector<int> &LLNodes);
        CBSNode createNode(const Map &map, const AgentSet &agentSet, const Config &config,
                           const Conflict &conflict, const std::vector<int> &costs,
                           ConstraintsSet &constraints, int id1, int id2,
                           const Node &pos1, const Node &pos2,
                           std::vector<std::list<Node>::iterator> &starts,
                           std::vector<std::list<Node>::iterator> &ends,
                           ConflictAvoidanceTable &CAT, ConflictSet &conflictSet,
                           std::vector<double> &lb,
                           std::vector<int> &LLExpansions, std::vector<int> &LLNodes, CBSNode *parentPtr);
        void getOccupiedNodes(std::vector<SIPPNode> &nodes,
                              std::list<Node>::iterator it, std::list<Node>::iterator end,
                              int agentId, std::vector<int> &lastStart);

        void getConflicts(const Map &map, const AgentSet &agentSet,
                          const ConstraintsSet &constraints, const std::vector<int> &costs,
                          std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
                          const std::vector<std::list<Node>::iterator> &starts,
                          const std::vector<std::list<Node>::iterator> &ends,
                          std::map<std::pair<int, int>, std::set<IntervalBoundary>> &boundaries,
                          const std::vector<SIPPNode> &nodes, int curAgentId,
                          ConflictSet &conflictSet, bool getAllConflicts, bool withCardinalConflicts);

        void addBoundaries(std::map<std::pair<int, int>, std::set<IntervalBoundary>> &boundaries,
                           const std::vector<SIPPNode> &nodes, int curAgentId);


        SearchType*                     search;
};

#endif // CONFLICT_BASED_SEARCH_H
