#include "node.h"
#include <vector>
#include <unordered_map>

struct Conflict {
    int id1, id2;
    Node pos1, pos2;
    int time;
    int minIncrease;
    bool edgeConflict;
    bool conflictFound;

    Conflict(bool ConflictFound) {
        conflictFound = ConflictFound;
    }

    Conflict(int Id1, int Id2, Node Pos1, Node Pos2, int Time, bool EdgeConflict) {
        id1 = Id1;
        id2 = Id2;
        pos1 = Pos1;
        pos2 = Pos2;
        time = Time;
        edgeConflict = EdgeConflict;
        conflictFound = true;
        minIncrease = 0;
    }

    bool operator<(const Conflict &rhs) const {
        return std::tuple<int, int, int, int, int, int, int, int>(
                    -minIncrease, time, id1, id2, pos1.i, pos1.j, pos2.i, pos2.j) <
                std::tuple<int, int, int, int, int, int, int, int>(
                    -rhs.minIncrease, rhs.time, rhs.id1, rhs.id2, rhs.pos1.i, rhs.pos1.j, rhs.pos2.i, rhs.pos2.j);
    }
};



