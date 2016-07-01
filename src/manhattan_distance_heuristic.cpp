#include "mha_global_planner/manhattan_distance_heuristic.h"

namespace mha_global_planner {

ManhattanDistanceHeuristic::ManhattanDistanceHeuristic(
    EnvironmentNAVXYTHETALAT* environment)
    : EmbeddedHeuristic(environment), environment_(environment)
{
}

int ManhattanDistanceHeuristic::GetGoalHeuristic(int state_id)
{
  return environment_->GetGoalHeuristic(state_id);
}

int ManhattanDistanceHeuristic::GetStartHeuristic(int state_id)
{
  return environment_->GetStartHeuristic(state_id);
}

int ManhattanDistanceHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
  return environment_->GetFromToHeuristic(from_id, to_id);
}
}
