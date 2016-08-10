#pragma once

#include "mha_global_planner/hashable_cell.h"

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sbpl/headers.h>
#include <unordered_set>

namespace mha_global_planner {

class AvoidSquareHeuristic : public EmbeddedHeuristic {
 public:
  void initialize();

  AvoidSquareHeuristic(EnvironmentNAVXYTHETALAT* environment,
                            float nominalvel_mpersecs);

  int GetGoalHeuristic(int state_id);

 private:
  float map_resolution_;
  float nominalvel_mpersecs_;

  EnvironmentNAVXYTHETALAT* environment_;
};
}
