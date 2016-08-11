#include "mha_global_planner/demo_path_distance_heuristic.h"
#include "mha_global_planner/hashable_cell.h"

namespace mha_global_planner {

DemoPathDistanceHeuristic::DemoPathDistanceHeuristic(
    EnvironmentNAVXYTHETALAT* environment, float nominalvel_mpersecs)
    : BaseHeuristic(environment, nominalvel_mpersecs) {}

void DemoPathDistanceHeuristic::initialize(std::string name) {
  BaseHeuristic::initialize(name, "demo_path_distance");
  demo_path_sub_ = private_nh_.subscribe(
      "demo_path", 10, &DemoPathDistanceHeuristic::demoPathCallback, this);
}

int DemoPathDistanceHeuristic::heuristicValue(int current_state_id) {
  // we calculate euclidean from our current position
  // to nearest cell where a demonstration has passed through
  int robot_x, robot_y, robot_theta;
  environment_->GetCoordFromState(current_state_id, robot_x, robot_y,
                                  robot_theta);

  float min_dist = std::numeric_limits<float>::max();
  int min_dist_cell_x;
  int min_dist_cell_y;
  for (const auto& cell : path_cell_set_) {
    float sqr_dist = (cell.x - robot_x) * (cell.x - robot_x) + (cell.y - robot_y) * (cell.y - robot_y);
    if (sqr_dist < min_dist) {
      min_dist = sqr_dist;
      min_dist_cell_x = cell.x;
      min_dist_cell_y = cell.y;
    }
  }

  //anything greater than 1 meter away we say doesn't matter
  if (min_dist >= 1)
  {
    return 100000;
  }

  // dist here is in cells so convert meters, then millimeters
  // then divide by speed. result is seconds/1000 as the units
  // this is only done to match how mha does it internally
  int cost = (1000 * min_dist * map_resolution_) / nominalvel_mpersecs_;
  ROS_WARN_THROTTLE(1, "%i,%i running demo path, %i", robot_x, robot_y, cost);
  return cost;
}

void DemoPathDistanceHeuristic::demoPathCallback(const nav_msgs::Path& msg) {
  ROS_INFO_ONCE("Receiving demo paths!");
  // parse out all the cells and put them into a set
  for (auto pose : msg.poses) {
    HashableCell cell;
    // pose x and y are in meters
    // we want two cells to be the same if they're in the same map
    // square, so divide by our map resolution
    cell.x = (int)(pose.pose.position.x / map_resolution_);
    cell.y = (int)(pose.pose.position.y / map_resolution_);
    path_cell_set_.insert(cell);
  }
}
}
