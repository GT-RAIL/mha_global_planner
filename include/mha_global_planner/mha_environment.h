#pragma once

#include <sbpl/headers.h>

namespace mha_global_planner {
class MhaEnvironment : public DiscreteSpaceInformation {
 public:

   MhaEnvironment();

  /**
   * \brief heuristic estimate from state FromStateID to state ToStateID
   */
  virtual int GetFromToHeuristic(int FromStateID, int ToStateID) override;
  /**
   * \brief heuristic estimate from state with stateID to goal state
   */
  virtual int GetGoalHeuristic(int stateID) override;

  /**
   * \brief heuristic estimate from start state to state with stateID
   */
  virtual int GetStartHeuristic(int stateID) override;

  /** \brief depending on the search used, it may call GetSuccs function
   * (for forward search) or GetPreds function (for backward search) or both
   * (for incremental search). At least one of these functions should be
   * implemented (otherwise, there will be no search to run) Some searches may
   * also use SetAllActionsandAllOutcomes or SetAllPreds functions if they keep
   * the pointers to successors (predecessors) but most searches do not require
   * this, so it is not necessary to support this.
   */
  virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV,
                        std::vector<int>* CostV) override;


  virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV) override;

  /**
   * \brief initialization environment from file (see .cfg files for examples)
   */
  virtual bool InitializeEnv(const char* sEnvFile) override;

  /**
   * \brief initialization of MDP data structure
   */
  virtual bool InitializeMDPCfg(MDPConfig* MDPCfg) override;

  /**
   * \brief prints environment config file
   */
  virtual void PrintEnv_Config(FILE* fOut) override;

  /**
   * \brief prints the state variables for a state with stateID
   */
  virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL) override;

  virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) override;

  virtual void SetAllPreds(CMDPSTATE* state) override;

  // returns the stateid if success, and -1 otherwise
  int SetGoal(double x_m, double y_m, double theta_rad);

  // returns the stateid if success, and -1 otherwise
  int SetStart(double x_m, double y_m, double theta_rad);

  virtual int SizeofCreatedEnv() override;
};
}
