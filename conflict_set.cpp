#include "conflict_set.h"

void ConflictSet::addCardinalConflict(Conflict &conflict) {
    cardinal.insert(conflict);
}

void ConflictSet::addSemiCardinalConflict(Conflict &conflict) {
    semiCardinal.insert(conflict);
}

void ConflictSet::addNonCardinalConflict(Conflict &conflict) {
    nonCardinal.insert(conflict);
}

void ConflictSet::replaceAgentConflicts(int agentId, ConflictSet &agentConflicts) {
    std::set<Conflict> newCardinal, newSemiCardinal, newNonCardinal;
    auto pred = [agentId](Conflict conflict) {return conflict.id1 != agentId && conflict.id2 != agentId;};
    std::copy_if(cardinal.begin(), cardinal.end(), std::inserter(newCardinal, newCardinal.begin()), pred);
    std::copy_if(semiCardinal.begin(), semiCardinal.end(), std::inserter(newSemiCardinal, newSemiCardinal.begin()), pred);
    std::copy_if(nonCardinal.begin(), nonCardinal.end(), std::inserter(newNonCardinal, newNonCardinal.begin()), pred);
    newCardinal.insert(agentConflicts.cardinal.begin(), agentConflicts.cardinal.end());
    newSemiCardinal.insert(agentConflicts.semiCardinal.begin(), agentConflicts.semiCardinal.end());
    newNonCardinal.insert(agentConflicts.nonCardinal.begin(), agentConflicts.nonCardinal.end());
    cardinal = newCardinal;
    semiCardinal = newSemiCardinal;
    nonCardinal = newNonCardinal;
}

bool ConflictSet::empty() {
    return cardinal.empty() && semiCardinal.empty() && nonCardinal.empty();
}

Conflict ConflictSet::getBestConflict() {
    if (!cardinal.empty()) {
        return *cardinal.begin();
    } else if (!semiCardinal.empty()) {
        return *semiCardinal.begin();
    }
    return *nonCardinal.begin();
}

int ConflictSet::getCardinalConflictCount() {
    return cardinal.size();
}

int ConflictSet::getConflictCount() {
    return cardinal.size() + semiCardinal.size() + nonCardinal.size();
}

/*std::vector<Conflict> ConflictSet::getCardinalConflicts() {
    return cardinal;
}*/

int ConflictSet::getMatchingHeuristic() {
    std::unordered_set<int> matched;
    int res = 0;
    for (auto conflict : cardinal) {
        if (matched.find(conflict.id1) == matched.end() && matched.find(conflict.id2) == matched.end()) {
            ++res;
            matched.insert(conflict.id1);
            matched.insert(conflict.id2);
        }
    }
    return res;
}

int ConflictSet::getConflictingPairsCount() {
    std::set<std::pair<int, int>> conflictingPairs;
    std::vector<std::set<Conflict>::iterator> begins = {nonCardinal.begin(), semiCardinal.begin(), cardinal.begin()};
    std::vector<std::set<Conflict>::iterator> ends = {nonCardinal.end(), semiCardinal.end(), cardinal.end()};
    for (int i = 0; i < 3; ++i) {
        for (auto it = begins[i]; it != ends[i]; ++it) {
            auto pair = std::make_pair(it->id1, it->id2);
            if (pair.first > pair.second) {
                std::swap(pair.first, pair.second);
            }
            conflictingPairs.insert(std::make_pair(it->id1, it->id2));
        }
    }
    return conflictingPairs.size();
}


