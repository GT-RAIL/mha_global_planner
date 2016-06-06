#include "mha_global_planner/mha_global_planner.h"

#include <pluginlib/class_list_macros.h>

// register plugin
PLUGINLIB_EXPORT_CLASS(mha_global_planner::MhaGlobalPlanner,
                       nav_core::BaseGlobalPlanner)

namespace mha_global_planner {

MhaGlobalPlanner::MhaGlobalPlanner() {}

MhaGlobalPlanner::MhaGlobalPlanner(std::string name,
                                   costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros);
}

void MhaGlobalPlanner::initialize(std::string name,
                                  costmap_2d::Costmap2DROS* costmap_ros) {}

bool MhaGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan) {
  plan.push_back(start);

  //...
  //plan.push_back(waypoints);
  //...

  plan.push_back(goal);

  return true;
}
};
