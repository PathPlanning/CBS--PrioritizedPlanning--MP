#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include "isearch.h"

template <typename NodeType = Node>
class Dijkstra : public ISearch<NodeType>
{
public:
    Dijkstra(const Primitives &mp, bool withTime = true) : ISearch<NodeType>(mp, withTime) {}
    virtual ~Dijkstra() {}
};
#endif
