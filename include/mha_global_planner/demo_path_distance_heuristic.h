#pragma once

#include "mha_global_planner/hashable_cell.h"

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sbpl/headers.h>
#include <unordered_set>

namespace mha_global_planner {

class DemoPathDistanceHeuristic : public EmbeddedHeuristic {
 public:
  void initialize();

  DemoPathDistanceHeuristic(EnvironmentNAVXYTHETALAT* environment,
                            float nominalvel_mpersecs);

  int GetGoalHeuristic(int state_id);
  int GetStartHeuristic(int state_id);
  int GetFromToHeuristic(int from_id, int to_id);

  void demoPathCallback(const nav_msgs::Path& msg);

 private:
  float map_resolution_;
  float nominalvel_mpersecs_;

  EnvironmentNAVXYTHETALAT* environment_;
  ros::Subscriber demo_path_sub_;
  std::unordered_set<HashableCell> path_cell_set_;
};
}
