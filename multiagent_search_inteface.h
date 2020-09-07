#ifndef MULTIAGENT_SEARCH_INTEFACE_H
#define MULTIAGENT_SEARCH_INTEFACE_H

#include "config.h"
#include "agent_set.h"
#include "map.h"
#include "multiagent_search_result.h"
#include "conflict_set.h"
#include "constraints_set.h"
#include <vector>

class MultiagentSearchInterface
{
public:
    virtual ~MultiagentSearchInterface(void);
    virtual MultiagentSearchResult startSearch(const Map &map, const Config &config, AgentSet &AgentSet) = 0;
    virtual ConflictSet findConflict(
            const Map &map, const AgentSet &agentSet,
            const ConstraintsSet &constraints, const std::vector<int> &costs,
            std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
            const std::vector<std::list<Node>::iterator> &starts,
            const std::vector<std::list<Node>::iterator> &ends,
            int agentId = -1, bool findAllConflicts = false,
            bool withCardinalConflicts = false) { return ConflictSet(); };
    virtual void clear() {
        agentsPaths.clear();
    }

protected:
    std::vector<std::vector<Node>>  agentsPaths;
};


#endif // MULTIAGENT_SEARCH_INTEFACE_H
