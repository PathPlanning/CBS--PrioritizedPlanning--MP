#include "constraints_set.h"
#include "motion_primitives.h"
#include <iostream>

void ConstraintsSet::addNodeConstraint(int i, int j, int time, int agentId) {
    nodeConstraints.insert(Constraint(i, j, time, agentId));
}

void ConstraintsSet::addGoalNodeConstraint(int i, int j, int time, int agentId) {
    goalNodeConstraints.insert(Constraint(i, j, time, agentId, -1, -1, true));
}

void ConstraintsSet::addPositiveConstraint(int i, int j, int time, int agentId, int prevI, int prevJ) {
    positiveConstraints.push_back(Constraint(i, j, time, agentId, prevI, prevJ));
}

void ConstraintsSet::addConstraint(Constraint &constraint) {
    if (constraint.goalNode) {
        goalNodeConstraints.insert(constraint);
    } else {
        nodeConstraints.insert(constraint);
    }
}

void ConstraintsSet::removeNodeConstraint(int i, int j, int time, int agentId) {
    nodeConstraints.erase(Constraint(i, j, time, agentId));
}

void ConstraintsSet::removeGoalNodeConstraint(int i, int j, int time, int agentId) {
    goalNodeConstraints.erase(Constraint(i, j, time, agentId, -1, -1, true));
}

ConstraintsSet ConstraintsSet::getAgentConstraints(int agentId) const {
    ConstraintsSet res;
    for (auto constraint : nodeConstraints) {
        if (constraint.agentId == agentId) {
            res.nodeConstraints.insert(constraint);
        }
    }
    for (auto constraint : goalNodeConstraints) {
        if (constraint.agentId == agentId) {
            res.goalNodeConstraints.insert(constraint);
        }
    }
    for (auto constraint : positiveConstraints) {
        if (constraint.agentId == agentId) {
            res.positiveConstraints.push_back(constraint);
        }
    }
    return res;
}

bool ConstraintsSet::hasConstraint(int i, int j, int agentId) const {
    auto it = nodeConstraints.lower_bound(Constraint(i, j, 0, agentId));
    if (it != nodeConstraints.end() && it->i == i && it->j == j) {
        return true;
    }
    return false;
}

bool ConstraintsSet::hasNodeConstraint(int i, int j, int time, int agentId, int duration) const {
    auto constraint = nodeConstraints.lower_bound(Constraint(i, j, time, agentId));
    if (constraint != nodeConstraints.end() && constraint->i == i && constraint->j == j && constraint->time < time + duration) {
        return true;
    }
    constraint = goalNodeConstraints.lower_bound(Constraint(i, j, 0));
    return constraint != goalNodeConstraints.end() && constraint->i == i && constraint->j == j && constraint->time <= time;
}

bool ConstraintsSet::hasFutureConstraint(int i, int j, int time, int agentId) const {
    std::set<Constraint>::iterator constraint = nodeConstraints.lower_bound(Constraint(i, j, time));
    if (constraint != nodeConstraints.end() && constraint->i == i && constraint->j == j) {
        return true;
    }
    constraint = goalNodeConstraints.lower_bound(Constraint(i, j, time));
    return constraint != goalNodeConstraints.end() && constraint->i == i && constraint->j == j;
}

std::vector<Constraint> ConstraintsSet::getPositiveConstraints() const {
    return positiveConstraints;
}

int ConstraintsSet::getFirstConstraintTime(int i, int j, int startTime, int agentId) const {
    int res = CN_INFINITY;
    std::set<Constraint>::iterator constraint = nodeConstraints.lower_bound(Constraint(i, j, startTime));
    if (constraint != nodeConstraints.end() && constraint->i == i && constraint->j == j) {
        res = constraint->time;
    }
    constraint = goalNodeConstraints.lower_bound(Constraint(i, j, startTime));
    if (constraint != goalNodeConstraints.end() && constraint->i == i &&
            constraint->j == j && constraint->time < res) {
        res = constraint->time;
    }
    return res;
}

int ConstraintsSet::getNewWaitTime(const Cell &cell, int startTime, int waitTime, int endTime, int agentId) const {
    auto it = nodeConstraints.lower_bound(Constraint(cell.i, cell.j,
                                                     startTime + waitTime + cell.interval.first, agentId));
    if (it == nodeConstraints.end() || it->i != cell.i || it->j != cell.j ||
            it->time > startTime + waitTime + cell.interval.second) {
        return waitTime;
    }
    for (it; it != nodeConstraints.end() && it->i == cell.i && it->j == cell.j &&
         it->time + 1 - cell.interval.first <= endTime; ++it) {
        if (it->time > startTime + waitTime + cell.interval.second) {
            return waitTime;
        }
        waitTime = it->time + 1 - startTime - cell.interval.first;
    }
    if (it == nodeConstraints.end() || it->i != cell.i || it->j != cell.j) {
        return waitTime;
    }
    return CN_INFINITY;
}

std::vector<std::pair<int, int>> ConstraintsSet::getSafeIntervals(int i, int j, int agentId,
                                                                  int startTime, int endTime,
                                                                  int duration) const {
    /*if (endTime > CN_INFINITY || (endTime > CN_INFINITY / 2 && endTime != CN_INFINITY)) {
        std::cout << "p\n";
    }*/

    int goalConstraintTime = -1;
    auto it = goalNodeConstraints.lower_bound(Constraint(i, j, 0));
    if (it != goalNodeConstraints.end() && it->i == i && it->j == j) {
        goalConstraintTime = it->time;
        if (goalConstraintTime <= endTime) {
            endTime = goalConstraintTime - duration;
        }
        if (endTime < startTime) {
            return {};
        }
    }

    int beg = 0;
    it = nodeConstraints.lower_bound(Constraint(i, j, startTime + duration, agentId));
    if (it != nodeConstraints.begin()) {
        auto pr = std::prev(it);
        if (pr->i == i && pr->j == j) {
            beg = pr->time + 1;
        }
    }

    std::vector<std::pair<int, int>> res;
    auto end = nodeConstraints.lower_bound(Constraint(i, j, endTime + duration, agentId));
    for (it; it != end; ++it) {
        if (it->time >= beg + duration) {
            res.push_back(std::make_pair(beg, it->time - duration));
        }
        beg = it->time + it->dur;
    }

    if (beg <= endTime) {
        endTime = CN_INFINITY;
        if (goalConstraintTime != -1) {
            endTime = goalConstraintTime - duration;
        }
        if (end != nodeConstraints.end() && end->i == i && end->j == j && end->time - duration < endTime) {
            endTime = end->time - duration;
        }
        if (beg <= endTime) {
            res.push_back(std::make_pair(beg, endTime));
        }
    }
    return res;
}

