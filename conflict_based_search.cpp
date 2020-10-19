#include "conflict_based_search.h"

int CBSNode::curId = 0;

template<typename SearchType>
ConflictBasedSearch<SearchType>::ConflictBasedSearch()
{
    search = nullptr;
}

template<typename SearchType>
ConflictBasedSearch<SearchType>::ConflictBasedSearch (SearchType *Search)
{
    search = Search;
    mp = search->getMP();
}

template<typename SearchType>
ConflictBasedSearch<SearchType>::~ConflictBasedSearch()
{
    if (search)
        delete search;
}

template<typename SearchType>
void ConflictBasedSearch<SearchType>::getOccupiedNodes(std::vector<SIPPNode> &nodes,
                                                         std::list<Node>::iterator it,
                                                         std::list<Node>::iterator end,
                                                         int agentId, std::vector<int> &lastStart) {
    if (std::next(it) != end) {
        Primitive pr = this->mp->getPrimitive(std::next(it)->primitiveId);
        auto cells = pr.getCells();
        for (auto cell : cells) {
            SIPPNode node(it->i + cell.i, it->j + cell.j);
            int start = std::next(it)->g - pr.intDuration;
            node.startTime = start + cell.interval.first;
            node.endTime = start + cell.interval.second;
            nodes.push_back(node);
        }
    } else {
        SIPPNode node(it->i, it->j, nullptr, it->g);
        node.endTime = CN_INFINITY;
        nodes.push_back(node);
    }

    nodes[0].startTime = lastStart[agentId];
    lastStart[agentId] = nodes.back().startTime;
    if (nodes.back().endTime != CN_INFINITY) {
        nodes.pop_back();
    }
}

template<typename SearchType>
void ConflictBasedSearch<SearchType>::getConflicts(const Map &map, const AgentSet &agentSet,
                                                     const ConstraintsSet &constraints, const std::vector<int> &costs,
                                                     std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
                                                     const std::vector<std::list<Node>::iterator> &starts,
                                                     const std::vector<std::list<Node>::iterator> &ends,
                                                     std::map<std::pair<int, int>, std::set<IntervalBoundary>> &boundaries,
                                                     const std::vector<SIPPNode> &nodes, int curAgentId,
                                                     ConflictSet &conflictSet, bool getAllConflicts, bool withCardinalConflicts) {
    for (auto node : nodes) {
        auto pair = std::make_pair(node.i, node.j);
        auto cellBoundaries = boundaries.find(pair);
        if (cellBoundaries != boundaries.end()) {
            std::map<int, int> startTime;
            std::vector<Conflict> conflicts;
            auto it = cellBoundaries->second.begin();
            auto end = cellBoundaries->second.upper_bound(IntervalBoundary(node.endTime, 0, true));
            for (it; it != end; ++it) {
                if (it->end) {
                    if (it->time >= node.startTime) {
                        conflicts.push_back(Conflict(curAgentId, it->id, node, node, it->time, false));
                    }
                    if (startTime.find(it->id) != startTime.end()) {
                        startTime.erase(it->id);
                    }
                } else {
                    startTime[it->id] = it->time;
                }
            }
            for (auto pair : startTime) {
                conflicts.push_back(Conflict(curAgentId, pair.first, node, node, node.endTime, false));
            }

            for (auto conflict : conflicts) {
                if (withCardinalConflicts) {
                    std::vector<int> inc;
                    for (auto id : {conflict.id1, conflict.id2}) {
                        Constraint constraint(conflict.pos1.i, conflict.pos1.j, conflict.time, id);
                        ConstraintsSet agentConstraints = constraints.getAgentConstraints(id);
                        agentConstraints.addConstraint(constraint);
                        SearchResult searchResult = this->search->startSearch(map, agentSet, starts[id]->i, starts[id]->j,
                                                                              std::prev(ends[id])->i, std::prev(ends[id])->j,
                                                                              0, -1, -1, agentConstraints);
                        if (!searchResult.pathfound) {
                            continue;
                        }
                        inc.push_back(std::max(searchResult.pathlength - costs[id], 0));
                        LLExpansions.push_back(searchResult.nodesexpanded);
                        LLNodes.push_back(searchResult.nodescreated);
                    }
                    for (; inc.size() < 2; inc.push_back(CN_INFINITY));
                    if (inc[0] > 0 && inc[1] > 0) {
                        conflict.minIncrease = std::min(inc[0], inc[1]);
                        conflictSet.addCardinalConflict(conflict);
                        if (!getAllConflicts) {
                            return;
                        }
                    } else if (inc[0] > 0 || inc[1] > 0) {
                        conflictSet.addSemiCardinalConflict(conflict);
                    } else {
                        conflictSet.addNonCardinalConflict(conflict);
                    }
                } else {
                    conflictSet.addNonCardinalConflict(conflict);
                    if (!getAllConflicts) {
                        return;
                    }
                }
            }
        }
    }
}

template<typename SearchType>
void ConflictBasedSearch<SearchType>::addBoundaries(std::map<std::pair<int, int>, std::set<IntervalBoundary>> &boundaries,
                                                      const std::vector<SIPPNode> &nodes, int curAgentId) {
    for (auto node : nodes) {
        auto pair = std::make_pair(node.i, node.j);
        if (boundaries.find(pair) == boundaries.end()) {
            boundaries[pair] = {};
        }
        boundaries[pair].insert(IntervalBoundary(node.startTime, curAgentId, false));
        boundaries[pair].insert(IntervalBoundary(node.endTime, curAgentId, true));
    }
}

template<typename SearchType>
ConflictSet ConflictBasedSearch<SearchType>::findConflict(
        const Map &map, const AgentSet &agentSet,
        const ConstraintsSet &constraints, const std::vector<int> &costs,
        std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
        const std::vector<std::list<Node>::iterator> &starts,
        const std::vector<std::list<Node>::iterator> &ends,
        int agentId, bool findAllConflicts,
        bool withCardinalConflicts) {
    ConflictSet conflictSet;
    std::vector<std::list<Node>::iterator> iters = starts;
    std::set<int> notFinished;
    std::map<std::pair<int, int>, std::set<IntervalBoundary>> boundaries;
    std::vector<int> lastStart(starts.size(), 0);

    for (int i = 0; i < starts.size(); ++i) {
        if (i != agentId) {
            notFinished.insert(i);
        }
    }
    if (agentId != -1) {
        for (auto it = starts[agentId]; it != ends[agentId]; ++it) {
            std::vector<SIPPNode> nodes;
            getOccupiedNodes(nodes, it, ends[agentId], agentId, lastStart);
            addBoundaries(boundaries, nodes, agentId);
        }
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    for (int i = 0; !notFinished.empty(); ++i) {
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > 30000) {
            return conflictSet;
        }

        std::set<int> newNotFinished;
        for (int j : notFinished) {
            std::vector<SIPPNode> nodes;
            if (std::next(iters[j]) != ends[j]) {
                newNotFinished.insert(j);
            }
            getOccupiedNodes(nodes, iters[j], ends[j], j, lastStart);
            getConflicts(map, agentSet, constraints, costs, LLExpansions, LLNodes, starts, ends,
                        boundaries, nodes, j, conflictSet, findAllConflicts, withCardinalConflicts);
            if (!findAllConflicts && !conflictSet.empty()) {
                return conflictSet;
            }
            if (agentId == -1) {
                addBoundaries(boundaries, nodes, j);
            }
            ++iters[j];
        }
        notFinished = newNotFinished;
    }
    return conflictSet;
}

template<typename SearchType>
CBSNode ConflictBasedSearch<SearchType>::createNode(const Map &map, const AgentSet &agentSet, const Config &config,
                                        const Conflict &conflict, const std::vector<int> &costs,
                                        ConstraintsSet &constraints, int id1, int id2,
                                        const Node &pos1, const Node &pos2,
                                        std::vector<std::list<Node>::iterator> &starts,
                                        std::vector<std::list<Node>::iterator> &ends,
                                        ConflictAvoidanceTable &CAT, ConflictSet &conflictSet,
                                        std::vector<double> &lb,
                                        std::vector<int> &LLExpansions, std::vector<int> &LLNodes,
                                        CBSNode *parentPtr) {
    Constraint constraint(pos1.i, pos1.j, conflict.time, id1);
    if (conflict.edgeConflict) {
        constraint.prev_i = pos2.i;
        constraint.prev_j = pos2.j;
    }

    ConstraintsSet agentConstraints = constraints.getAgentConstraints(id1);
    agentConstraints.addConstraint(constraint);

    Agent agent = agentSet.getAgent(id1);
    if (config.withCAT || config.withFocalSearch == true) {
        CAT.removeAgentPath(starts[id1], ends[id1], mp);
    }

    double oldLb = lb[id1];
    SearchResult searchResult = search->startSearch(map, agentSet, agent.getStart_i(), agent.getStart_j(),
                                                    agent.getGoal_i(), agent.getGoal_j(), 0, -1, -1,
                                                    agentConstraints, config.withCAT, CAT);
    if (config.withCAT || config.withFocalSearch == true) {
        CAT.addAgentPath(starts[id1], ends[id1], mp);
    }
    if (!searchResult.pathfound) {
        return CBSNode(false);
    }
    std::list<Node> newPath = *searchResult.lppath;
    LLNodes.push_back(searchResult.nodescreated);
    LLExpansions.push_back(searchResult.nodesexpanded);
    lb[id1] = searchResult.minF;

    CBSNode node;
    node.paths[id1] = newPath;
    for (int i = 0; i < costs.size(); ++i) {
        if (i == id1) {
            node.cost += newPath.back().g;
        } else {
            node.cost += costs[i];
        }
    }
    node.constraint = constraint;
    node.parent = parentPtr;

    auto oldStart = starts[id1], oldEnd = ends[id1];
    starts[id1] = node.paths[id1].begin();
    ends[id1] = node.paths[id1].end();
    if (config.storeConflicts) {
        ConflictSet agentConflicts = findConflict(map, agentSet, constraints, costs, LLExpansions, LLNodes,
                                                  starts, ends, id1, true, config.withCardinalConflicts);
        node.conflictSet = conflictSet;
        node.conflictSet.replaceAgentConflicts(id1, agentConflicts);
    }
    if (config.withFocalSearch) {
        node.hc = node.conflictSet.getConflictingPairsCount();
        node.sumLb = 0;
        for (auto x : lb) {
            node.sumLb += x;
        }
        node.lb[id1] = lb[id1];
        lb[id1] = oldLb;
    }

    if (config.withMatchingHeuristic) {
        node.H = node.conflictSet.getMatchingHeuristic();
    }

    starts[id1] = oldStart;
    ends[id1] = oldEnd;

    node.G = node.H + node.cost;
    return node;
}

template<typename SearchType>
MultiagentSearchResult ConflictBasedSearch<SearchType>::startSearch(const Map &map, const Config &config, AgentSet &agentSet) {
    // std::cout << agentSet.getAgentCount() << std::endl;
    CBSNode::curId = 0;
    ISearch<>::T = 0;

    if (config.withPerfectHeuristic) { //&& !(config.lowLevel == CN_SP_ST_TKN && config.neighDegree > 2)) {
        int prevAgentCount;
        if (agentSet.getAgentCount() == config.minAgents) {
            prevAgentCount = 0;
        } else {
            prevAgentCount = agentSet.getAgentCount() - config.agentsStep;
        }
        search->getPerfectHeuristic(map, agentSet, *search->getMP(), prevAgentCount);
    }

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    auto focalCmp = [](const CBSNode &lhs, const CBSNode &rhs) {
        return lhs.hc < rhs.hc || lhs.hc == rhs.hc && lhs < rhs;
    };
    std::set<CBSNode, decltype(focalCmp)> focal(focalCmp);
    std::multiset<double> sumLb;

    CBSNode root;
    int agentCount = agentSet.getAgentCount();
    std::vector<std::list<Node>::iterator> starts(agentCount), ends(agentCount);
    std::vector<int> initialCosts(agentCount, 0);
    for (int i = 0; i < agentSet.getAgentCount(); ++i) {
        //std::cout << i << std::endl;
        Agent agent = agentSet.getAgent(i);
        Astar<> astar(*search->getMP(), false);
        SearchResult searchResult;
        searchResult = astar.startSearch(map, agentSet, agent.getStart_i(), agent.getStart_j(),
                                                        agent.getGoal_i(), agent.getGoal_j());
        if (!searchResult.pathfound) {
            std::cout << "fail" << std::endl;
        }
        root.cost += searchResult.pathlength;
        initialCosts[i] = searchResult.pathlength;
        root.paths[i] = *searchResult.lppath;

        starts[i] = root.paths[i].begin();
        ends[i] = root.paths[i].end();
        if (config.withFocalSearch) {
            root.lb[i] = searchResult.minF;
            root.sumLb += searchResult.minF;
        }
    }

    std::vector<int> LLExpansions, LLNodes;
    if (config.storeConflicts) {
        root.conflictSet = findConflict(map, agentSet, ConstraintsSet(), initialCosts, LLExpansions, LLNodes,
                                        starts, ends, -1, true, config.withCardinalConflicts);
        if (config.withFocalSearch) {
            root.hc = root.conflictSet.getConflictingPairsCount();
            sumLb.insert(root.sumLb);
        }
    }

    MultiagentSearchResult result(false);
    std::set<CBSNode> open = {root};
    std::list<CBSNode> close;

    int t = 0;
    while (!open.empty() || !focal.empty()) {
        //std::cout << ISearch<>::T << std::endl;
        ++t;
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count() > config.maxTime) {
            result.pathfound = false;
            break;
        }

        CBSNode cur;
        if (config.withFocalSearch) {
            double threshold = *sumLb.begin() * config.focalW;
            auto it = open.begin();
            for (it; it != open.end() && it->cost <= threshold; ++it) {
                focal.insert(*it);
            }
            open.erase(open.begin(), it);
            cur = *focal.begin();
            focal.erase(cur);
            sumLb.erase(sumLb.find(cur.sumLb));
        } else {
            cur = *open.begin();
            open.erase(cur);
        }
        close.push_back(cur);

        std::vector<int> costs(agentCount, 0);
        std::vector<bool> agentFound(agentCount, false);
        std::vector<std::list<Node>::iterator> starts(agentCount), ends(agentCount);
        ConstraintsSet constraints;
        ConflictAvoidanceTable CAT;
        std::vector<double> lb(agentCount);
        for (CBSNode *ptr = &cur; ptr != nullptr; ptr = ptr->parent) {
            for (auto it = ptr->paths.begin(); it != ptr->paths.end(); ++it) {
                if (!agentFound[it->first]) {
                    starts[it->first] = it->second.begin();
                    ends[it->first] = it->second.end();
                    costs[it->first] = it->second.back().g;
                    agentFound[it->first] = true;

                    if (config.withCAT || config.withFocalSearch == true) {
                        CAT.addAgentPath(starts[it->first], ends[it->first], mp);
                    }
                    if (config.withFocalSearch) {
                        lb[it->first] = ptr->lb[it->first];
                    }
                }
            }
            if (ptr->id != 0) {
                constraints.addConstraint(ptr->constraint);
            }
        }

        ConflictSet conflictSet;
        if (!config.storeConflicts) {
            conflictSet = findConflict(map, agentSet, constraints, costs, LLExpansions, LLNodes,
                        starts, ends, -1, false, config.withCardinalConflicts);
        } else {
            conflictSet = cur.conflictSet;
        }
        if (conflictSet.empty()) {
            agentsPaths.resize(agentCount);
            for (int i = 0; i < agentCount; ++i) {
                for (auto it = starts[i]; it != ends[i]; ++it) {
                    agentsPaths[i].push_back(*it);
                }
            }
            result.agentsPaths = &agentsPaths;
            result.pathfound = true;
            result.HLExpansions = close.size();
            result.HLNodes = open.size() + close.size() + focal.size();
            if (LLExpansions.empty()) {
                result.AvgLLExpansions = 0;
                result.AvgLLNodes = 0;
            } else {
                result.AvgLLExpansions = (double)std::accumulate(LLExpansions.begin(), LLExpansions.end(), 0) / LLExpansions.size();
                result.AvgLLNodes = (double)std::accumulate(LLNodes.begin(), LLNodes.end(), 0) / LLNodes.size();
            }

            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            int elapsedMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
            result.time = static_cast<double>(elapsedMilliseconds) / 1000;
            break;
        }

        Conflict conflict = conflictSet.getBestConflict();

        // std::cout << conflict.id1 << " " << conflict.id2 << " " << conflict.time << std::endl;
        //std::cout << cur.constraint.i << " " << cur.constraint.j << " " << cur.constraint.time << " "
        //          << cur.constraint.agentId << std::endl;

        /*if (config.withDisjointSplitting &&
                mdds[conflict.id1].getLayerSize(conflict.time) < mdds[conflict.id2].getLayerSize(conflict.time)) {
            std::swap(conflict.id1, conflict.id2);
            std::swap(conflict.pos1, conflict.pos2);
        }*/

        std::vector<CBSNode> children;
        CBSNode child1 = createNode(map, agentSet, config, conflict, costs, constraints,
                                   conflict.id1, conflict.id2, conflict.pos2, conflict.pos1,
                                   starts, ends, CAT, conflictSet, lb, LLExpansions, LLNodes, &close.back());
        if (child1.pathFound) {
            children.push_back(child1);
        }
        CBSNode child2 = createNode(map, agentSet, config, conflict, costs, constraints,
                           conflict.id2, conflict.id1, conflict.pos1, conflict.pos2,
                           starts, ends, CAT, conflictSet, lb, LLExpansions, LLNodes, &close.back());
        if (child2.pathFound) {
            children.push_back(child2);
        }

        /*for (auto child : children) {
            for (auto c : constraints.nodeConstraints) {
                if (c == child.constraint && c.agentId == child.constraint.agentId) {
                    std::cout << t << " q" << std::endl;
                }
            }
        }*/

        /*for (auto child : children) {
            auto newStarts = starts;
            auto newEnds = ends;
            newStarts[child.constraint.agentId] = child.paths.begin()->second.begin();
            newEnds[child.constraint.agentId] = child.paths.begin()->second.end();
            conflictSet = findConflict(newStarts, newEnds, -1, false, config.withCardinalConflicts, mdds);
            for (auto c : conflictSet.nonCardinal) {
                if (c.id1 == conflict.id1 && c.id2 == conflict.id2 &&
                        c.pos1 == conflict.pos1 && c.time == conflict.time) {
                    std::cout << t << " q\n";
                }
            }
        }*/

        for (auto child : children) {
            /*for (auto constraint : constraints.nodeConstraints) {
                if (constraint.agentId == child.paths.begin()->first) {
                    for (auto it = child.paths.begin()->second.begin(); it != child.paths.begin()->second.end(); ++it) {
                        if (std::next(it) != child.paths.begin()->second.end()) {
                            auto pr = this->mp->getPrimitive(std::next(it)->primitiveId);
                            for (auto cell : pr.cells) {
                                if (it->i + cell.i == constraint.i && it->j + cell.j == constraint.j) {
                                    int start = std::next(it)->g - pr.intDuration + cell.interval.first;
                                    int end = std::next(it)->g - pr.intDuration + cell.interval.second;
                                    if (start <= constraint.time && constraint.time <= end) {
                                        std::cout << "1 " << t << std::endl;
                                    }
                                }
                            }
                        }
                    }
                }
            }*/

            /*for (auto constraint : constraints.nodeConstraints) {
                if (constraint.agentId == child.paths.begin()->first) {
                    auto it = child.paths.begin()->second.begin();
                    std::advance(it, std::min(constraint.time, int(child.paths.begin()->second.size()) - 1));
                    if (it->i == constraint.i && it->j == constraint.j) {
                        std::cout << "1 " << t << std::endl;
                    }
                }
            }
            for (auto constraint : constraints.edgeConstraints) {
                if (constraint.agentId == child.paths.begin()->first) {
                    auto it = child.paths.begin()->second.begin();
                    if (constraint.time > child.paths.begin()->second.size() - 1) {
                        continue;
                    }
                    std::advance(it, constraint.time);
                    if (it->i == constraint.i && it->j == constraint.j &&
                            std::prev(it)->i == constraint.prev_i && std::prev(it)->j == constraint.prev_j) {
                        std::cout << "2 " << t << std::endl;
                    }
                }
            }
            for (auto constraint : constraints.positiveConstraints) {
                if (constraint.agentId == child.paths.begin()->first) {
                    auto it = child.paths.begin()->second.begin();
                    std::advance(it, std::min(constraint.time, int(child.paths.begin()->second.size()) - 1));
                    if ((it->i != constraint.i || it->j != constraint.j)) {
                        std::cout << "3 " << t << std::endl;
                    }
                }
            }*/
        }

        bool bypass = false;
        if (children.size() == 2) {
            for (auto child : children) {
                if (child.pathFound && config.withBypassing && cur.cost == child.cost &&
                        conflictSet.getConflictCount() > child.conflictSet.getConflictCount()) {
                    open.insert(child);
                    bypass = true;
                    break;
                }
            }
        }
        if (!bypass && !children.empty()) {
            for (auto child : children) {
                open.insert(child);
                if (config.withFocalSearch) {
                    sumLb.insert(child.sumLb);
                }
            }
        }
    }
    //std::cout << close.size() + open.size() << std::endl;
    //std::cout << ISearch<TwoKNeighSIPPNode>::T << std::endl;
    return result;
}

template class ConflictBasedSearch<Astar<>>;
template class ConflictBasedSearch<SIPP<>>;
template class ConflictBasedSearch<ZeroSCIPP<>>;
template class ConflictBasedSearch<FocalSearch<>>;
template class ConflictBasedSearch<SCIPP<>>;
