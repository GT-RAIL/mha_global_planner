#include "mha_global_planner/mha_global_planner.h"

#include <sbpl/utils/mdpconfig.h>
#include <sbpl/planners/mhaplanner.h>
#include <sbpl/discrete_space_information/environment_navxythetalat.h>

#include <pluginlib/class_list_macros.h>

// register plugin
PLUGINLIB_EXPORT_CLASS(mha_global_planner::MhaGlobalPlanner,
                       nav_core::BaseGlobalPlanner)

namespace mha_global_planner {

MhaGlobalPlanner::MhaGlobalPlanner() {
  this->mha_planner_ = new MHAPlanner(environment_, anchor_heuristic_, heuristics_.data(), heuristics_.size());
}

MhaGlobalPlanner::MhaGlobalPlanner(std::string name,
                                   costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros);
}

void MhaGlobalPlanner::initialize(std::string name,
                                  costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    initialized_ = true;
  } else {
    ROS_WARN("This planner has already been initialized... doing nothing");
  }
}

bool MhaGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    ROS_ERROR(
        "The planner has not been initialized, please call initialize() to use "
        "the planner.");
    ROS_ERROR("note, this SHOULD have happened in the constructor.");
    return false;
  }

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2F, %.2f",
            start.pose.position.x, start.pose.position.y, goal.pose.position.x,
            goal.pose.position.y);

  plan.push_back(start);

  //START USING SBPL HERE

  int bRet = 0;
  double allocated_time_secs = 10.0; // in seconds
  double initialEpsilon = 3.0;
  MDPConfig MDPCfg;
  bool bsearchuntilfirstsolution = false;
  bool bforwardsearch = true;

  EnvironmentNAVXYTHETALAT environment_navxythetalat;

/*
 *  if (!environment_navxythetalat.InitializeEnv(envCfgFilename, perimeterptsV, motPrimFilename)) {
 *      throw SBPL_Exception("ERROR: InitializeEnv failed");
 *  }
 *
 *  // Initialize MDP Info
 *  if (!environment_navxythetalat.InitializeMDPCfg(&MDPCfg)) {
 *      throw SBPL_Exception("ERROR: InitializeMDPCfg failed");
 *  }
 *
 *  // plan a path
 *  vector<int> solution_stateIDs_V;
 */
  //...
  // plan.push_back(waypoints);
  //...

  plan.push_back(goal);

  return true;
}
};
