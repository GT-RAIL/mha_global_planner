#pragma once

#include "mha_global_planner/base_heuristic.h"

#include <nav_msgs/Path.h>
#include <unordered_set>

namespace mha_global_planner {

class DemoPathDistanceHeuristic : public BaseHeuristic {
 public:
  DemoPathDistanceHeuristic(EnvironmentNAVXYTHETALAT* environment,
                            float nominalvel_mpersecs);
  void initialize(std::string name);
  int heuristicValue(int state_id);
  void demoPathCallback(const nav_msgs::Path& msg);

 private:
  ros::Subscriber demo_path_sub_;
  std::unordered_set<HashableCell> path_cell_set_;
};
}
