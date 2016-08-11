#pragma once

#include "mha_global_planner/hashable_cell.h"

#include <dynamic_reconfigure/server.h>
#include <mha_global_planner/MhaGlobalPlannerConfig.h>
#include <ros/ros.h>
#include <sbpl/headers.h>

namespace mha_global_planner {

class BaseHeuristic : public EmbeddedHeuristic {
 public:
  void initialize(std::string node_name, std::string heuristic_name);
  virtual void initialize(std::string name) = 0;

  BaseHeuristic(EnvironmentNAVXYTHETALAT* environment,
                float nominalvel_mpersecs);

  int GetGoalHeuristic(int state_id);

  virtual int heuristicValue(int state_id) = 0;

  bool enabled_;
  float map_resolution_;
  float nominalvel_mpersecs_;
  ros::NodeHandle private_nh_;

  dynamic_reconfigure::Server<MhaGlobalPlannerConfig>* dsrv_;

  EnvironmentNAVXYTHETALAT* environment_;

  void reconfigureCB(MhaGlobalPlannerConfig& config, uint32_t level);
};
}
