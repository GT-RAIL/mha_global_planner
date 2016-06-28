#pragma once

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include "sbpl/planners/mhaplanner.h"
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
  bool initialized_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  MHAPlanner* mha_planner_;
  DiscreteSpaceInformation* environment_;
  Heuristic* anchor_heuristic_;
  std::vector<Heuristic *> heuristics_;

};
};
