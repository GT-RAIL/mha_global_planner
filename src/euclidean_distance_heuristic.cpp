#include "mha_global_planner/euclidean_distance_heuristic.h"

namespace mha_global_planner {

EuclideanDistanceHeuristic::EuclideanDistanceHeuristic(
    EnvironmentNAVXYTHETALAT* environment)
    : EmbeddedHeuristic(environment), environment_(environment) {}

int EuclideanDistanceHeuristic::GetGoalHeuristic(int state_id) {
  return environment_->GetGoalHeuristic(state_id);
}

int EuclideanDistanceHeuristic::GetStartHeuristic(int state_id) {
  return environment_->GetStartHeuristic(state_id);
}

int EuclideanDistanceHeuristic::GetFromToHeuristic(int from_id, int to_id) {
  return environment_->GetFromToHeuristic(from_id, to_id);
}
}
