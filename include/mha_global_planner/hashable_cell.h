#pragma once

#include <boost/functional/hash.hpp>

typedef struct hashable_cell_t {
  int x;
  int y;
} HashableCell;

namespace std {
template <>
struct hash<HashableCell> {
  std::size_t operator()(const HashableCell& cell) const {
    std::size_t seed = 0;

    boost::hash_combine(seed, cell.x);
    boost::hash_combine(seed, cell.y);

    return seed;
  }
};
}

bool operator==(const HashableCell& lhs, const HashableCell& rhs);
