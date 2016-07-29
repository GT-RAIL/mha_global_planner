#pragma once

#include "mha_global_planner/demo_path_distance_heuristic.h"

#include <angles/angles.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <sbpl/headers.h>
#include <stdlib.h>

namespace mha_global_planner {

class MhaGlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  MhaGlobalPlanner();
  MhaGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  virtual bool makePlan(const geometry_msgs::PoseStamped& start,
                        const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan);

  virtual void initialize(std::string name,
                          costmap_2d::Costmap2DROS* costmap_ros);

 private:
  // \brief speed we assume the robot drives in meters per sec
  double nominalvel_mpersecs_;

  // \brief speed we assume the robot rotates sec per 45 deg rotation,
  double timetoturn45degsinplace_secs_;

  bool initialized_;
  int obst_cost_thresh_;
  int force_scratch_limit_;
  double initial_epsilon_;

  // \brief allowed planning time in seconds
  double allocated_time_;

  unsigned char lethal_obstacle_;
  unsigned char inscribed_inflated_obstacle_;
  unsigned char sbpl_cost_multiplier_;

  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  EnvironmentNAVXYTHETALAT* env_;
  MHAPlanner* mha_planner_;
  Heuristic* anchor_heuristic_;
  DemoPathDistanceHeuristic* demo_path_heuristic_;
  std::string primitive_filename_;
  std::vector<Heuristic*> heuristics_;
  ros::Publisher plan_pub_;

  unsigned char costMapCostToSBPLCost(unsigned char newcost);
};
}
