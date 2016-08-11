#include "mha_global_planner/avoid_square_heuristic.h"
#include "mha_global_planner/hashable_cell.h"

namespace mha_global_planner {

AvoidSquareHeuristic::AvoidSquareHeuristic(
    EnvironmentNAVXYTHETALAT* environment, float nominalvel_mpersecs)
    : BaseHeuristic(environment, nominalvel_mpersecs) {}

void AvoidSquareHeuristic::initialize(std::string name) {
  BaseHeuristic::initialize(name, "avoid_square");
}

int AvoidSquareHeuristic::heuristicValue(int current_state_id) {
  ROS_WARN_THROTTLE(10, "running avoid");
  // we calculate euclidean from our current position
  // to nearest cell where a demonstration has passed through
  int state_x, state_y, state_theta;
  environment_->GetCoordFromState(current_state_id, state_x, state_y,
                                  state_theta);
  double robot_x = state_x * map_resolution_;
  double robot_y = state_y * map_resolution_;
  double robot_theta = state_theta * map_resolution_;

  if (1 < robot_x && robot_x < 2.5 && 2 < robot_y && robot_y < 4.5) {
    return 100000;
  }
  return 0;
}
}
