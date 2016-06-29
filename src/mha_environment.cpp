#include "mha_global_planner/mha_environment.h"

namespace mha_global_planner {

MhaEnvironment::MhaEnvironment() {}

int MhaEnvironment::SetStart(double x_m, double y_m, double theta_rad){}

/**
 * \brief initialization environment from file (see .cfg files for examples)
 */
bool MhaEnvironment::InitializeEnv(const char* sEnvFile) {}

/**
 * \brief initialization of MDP data structure
 */
bool MhaEnvironment::InitializeMDPCfg(MDPConfig* MDPCfg) {}

/**
 * \brief heuristic estimate from state FromStateID to state ToStateID
 */
int MhaEnvironment::GetFromToHeuristic(int FromStateID, int ToStateID) {}

/**
 * \brief heuristic estimate from state with stateID to goal state
 */
int MhaEnvironment::GetGoalHeuristic(int stateID) {}

/**
 * \brief heuristic estimate from start state to state with stateID
 */
int MhaEnvironment::GetStartHeuristic(int stateID) {}

/** \brief depending on the search used, it may call GetSuccs function
 * (for forward search) or GetPreds function (for backward search) or both
 * (for incremental search). At least one of these functions should be
 * implemented (otherwise, there will be no search to run) Some searches may
 * also use SetAllActionsandAllOutcomes or SetAllPreds functions if they keep
 * the pointers to successors (predecessors) but most searches do not require
 * this, so it is not necessary to support this.
 */
void MhaEnvironment::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV,
                              std::vector<int>* CostV) {}

void MhaEnvironment::GetPreds(int TargetStateID, std::vector<int>* PredIDV,
              std::vector<int>* CostV);

void MhaEnvironment::SetAllPreds(CMDPSTATE* state){}

int MhaEnvironment::SizeofCreatedEnv(){}

/**
 * \brief prints the state variables for a state with stateID
 */
void MhaEnvironment::PrintState(int stateID, bool bVerbose, FILE* fOut = NULL){}

/**
 * \brief prints environment config file
 */
void MhaEnvironment::PrintEnv_Config(FILE* fOut){}

void MhaEnvironment::SetAllActionsandAllOutcomes(CMDPSTATE* state){}
};
}
