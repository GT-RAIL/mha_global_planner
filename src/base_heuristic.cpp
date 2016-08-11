#include "mha_global_planner/base_heuristic.h"

namespace mha_global_planner {

BaseHeuristic::BaseHeuristic(EnvironmentNAVXYTHETALAT* environment,
                             float nominalvel_mpersecs)
    : EmbeddedHeuristic(environment),
      enabled_(true),
      environment_(environment),
      nominalvel_mpersecs_(nominalvel_mpersecs) {}

void BaseHeuristic::initialize(std::string node_name,
                               std::string heuristic_name) {
  private_nh_ = ros::NodeHandle("~/" + node_name + "/" + heuristic_name);
  private_nh_.param<float>("global_costmap_resolution", map_resolution_, 0.05);

  dsrv_ = new dynamic_reconfigure::Server<MhaGlobalPlannerConfig>(private_nh_);
  dynamic_reconfigure::Server<MhaGlobalPlannerConfig>::CallbackType cb;
  cb = boost::bind(&BaseHeuristic::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
  ROS_INFO("set up dynamic_reconfigure for %s",
           private_nh_.getNamespace().c_str());
}

int BaseHeuristic::GetGoalHeuristic(int state_id) {
  if (!enabled_) {
    return 0;
  }
  return heuristicValue(state_id);
}

void BaseHeuristic::reconfigureCB(MhaGlobalPlannerConfig& config,
                                  uint32_t level) {
  this->enabled_ = config.enabled;
}
}
