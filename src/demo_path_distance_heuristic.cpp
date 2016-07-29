#include "mha_global_planner/demo_path_distance_heuristic.h"
#include "mha_global_planner/hashable_cell.h"

namespace mha_global_planner {

DemoPathDistanceHeuristic::DemoPathDistanceHeuristic(
    EnvironmentNAVXYTHETALAT* environment, float nominalvel_mpersecs)
    : EmbeddedHeuristic(environment),
      environment_(environment),
      nominalvel_mpersecs_(nominalvel_mpersecs) {}

void DemoPathDistanceHeuristic::initialize() {
  ros::NodeHandle private_nh("~");
  demo_path_sub_ = private_nh.subscribe(
      "demo_path", 10, &DemoPathDistanceHeuristic::demoPathCallback, this);
  // TODO: don't forget to remap this in your launch file
  private_nh.param<float>("global_costmap_resolution", map_resolution_, 0.05);
}

int DemoPathDistanceHeuristic::GetStartHeuristic(int current_state_id) {
  // not used in MHA*
  return 0;
}

int DemoPathDistanceHeuristic::GetGoalHeuristic(int current_state_id) {
  // we calculate euclidean from our current position
  // to nearest cell where a demonstration has passed through
  int robot_x, robot_y, robot_theta;
  environment_->GetCoordFromState(current_state_id, robot_x, robot_y,
                                  robot_theta);

  float min_dist = std::numeric_limits<float>::max();
  for (const auto& cell : path_cell_set_) {
    float dist = std::sqrt((cell.x - robot_x) * (cell.x - robot_x) +
                           (cell.y - robot_y) * (cell.y - robot_y));
    if (dist < min_dist) {
      min_dist = dist;
    }
  }

  // dist here is in meters so convert to time
  return min_dist * nominalvel_mpersecs_;
}

int DemoPathDistanceHeuristic::GetFromToHeuristic(int form_id, int to_id) {
  // not used in MHA*
  return 0;
}

void DemoPathDistanceHeuristic::demoPathCallback(const nav_msgs::Path& msg) {
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
