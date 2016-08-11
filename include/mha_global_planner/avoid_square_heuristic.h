#pragma once

#include "mha_global_planner/base_heuristic.h"

namespace mha_global_planner {

class AvoidSquareHeuristic : public BaseHeuristic {
 public:
  AvoidSquareHeuristic(EnvironmentNAVXYTHETALAT* environment,
                       float nominalvel_mpersecs);
  void initialize(std::string name);
  int heuristicValue(int state_id);
};
}
