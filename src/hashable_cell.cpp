#include "mha_global_planner/hashable_cell.h"

bool operator==(const HashableCell& lhs, const HashableCell& rhs) {
  return (lhs.x == rhs.x) && (lhs.y == rhs.y);
}
