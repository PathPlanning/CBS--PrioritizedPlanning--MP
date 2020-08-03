#include "mp_conflict_based_search.h"

template<typename SearchType>
MPConflictBasedSearch<SearchType>::~MPConflictBasedSearch() {}

template class MPConflictBasedSearch<TwoKNeighSIPP<>>;

