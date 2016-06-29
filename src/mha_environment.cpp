#include "mha_global_planner/mha_environment.h"

namespace mha_global_planner {

MhaEnvironment::MhaEnvironment() {}

int MhaEnvironment::GetFromToHeuristic(int FromStateID, int ToStateID) {}

int MhaEnvironment::GetGoalHeuristic(int stateID) {}

int MhaEnvironment::GetStartHeuristic(int stateID) {}

void MhaEnvironment::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV) {}

void MhaEnvironment::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);

bool MhaEnvironment::InitializeEnv(const char* sEnvFile) {}

bool MhaEnvironment::InitializeMDPCfg(MDPConfig* MDPCfg) {}

void MhaEnvironment::PrintEnv_Config(FILE* fOut){}

void MhaEnvironment::PrintState(int stateID, bool bVerbose, FILE* fOut = NULL){}

void MhaEnvironment::SetAllActionsandAllOutcomes(CMDPSTATE* state){}

void MhaEnvironment::SetAllPreds(CMDPSTATE* state){}

int MhaEnvironment::SetGoal(double x_m, double y_m, double theta_rad){}

int MhaEnvironment::SetStart(double x_m, double y_m, double theta_rad){}

int MhaEnvironment::SizeofCreatedEnv(){}
};
}
