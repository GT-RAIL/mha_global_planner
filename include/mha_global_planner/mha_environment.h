#pragma once

#include <sbpl/headers.h>

namespace mha_global_planner {

struct EnvMhaHashEntry_t
{
    int stateID;
    int X;
    int Y;
    char Theta;
    int iteration;
};

//configuration parameters
struct EnvMhaConfig_t
{
    int EnvWidth_c;
    int EnvHeight_c;
    int NumThetaDirs;
    int StartX_c;
    int StartY_c;
    int StartTheta;
    int EndX_c;
    int EndY_c;
    int EndTheta;
    unsigned char** Grid2D;

    std::vector<double> ThetaDirs;
    int StartTheta_rad;
    int EndTheta_rad;
    double min_turning_radius_m;

    // the value at which and above which cells are obstacles in the maps sent from outside
    // the default is defined above
    unsigned char obsthresh;

    // the value at which and above which until obsthresh (not including it)
    // cells have the nearest obstacle at distance smaller than or equal to
    // the inner circle of the robot. In other words, the robot is definitely
    // colliding with the obstacle, independently of its orientation
    // if no such cost is known, then it should be set to obsthresh (if center
    // of the robot collides with obstacle, then the whole robot collides with
    // it independently of its rotation)
    unsigned char cost_inscribed_thresh;

    // the value at which and above which until cost_inscribed_thresh (not including it) cells
    // **may** have a nearest osbtacle within the distance that is in between
    // the robot inner circle and the robot outer circle
    // any cost below this value means that the robot will NOT collide with any
    // obstacle, independently of its orientation
    // if no such cost is known, then it should be set to 0 or -1 (then no cell
    // cost will be lower than it, and therefore the robot's footprint will
    // always be checked)
    int cost_possibly_circumscribed_thresh; // it has to be integer, because -1 means that it is not provided.

    double nominalvel_mpersecs;

    //double nominalangvel_radpersecs;

    double timetoturn45degsinplace_secs;

    double cellsize_m;

#define GRID_DIRECTIONS 8
    int dXY[GRID_DIRECTIONS][2];

    //array of actions, ActionsV[i][j] - jth action for sourcetheta = i
    EnvMhaAction_t** ActionsV;
    //PredActionsV[i] - vector of pointers to the actions that result in a state with theta = i
    std::vector<EnvMhaAction_t*>* PredActionsV;

    int actionwidth; //number of motion primitives
    std::vector<SBPL_xytheta_mprimitive> mprimV;

    std::vector<sbpl_2Dpt_t> FootprintPolygon;
};

//variables that dynamically change (e.g., array of states, ...)
typedef struct EnvMha
{
    EnvMha()
    {
        Coord2StateIDHashTable = NULL;
    }

    int startstateid;
    int goalstateid;

    bool bInitialized;

    //hash table of size x_size*y_size. Maps from coords to stateId
    int HashTableSize;
    std::vector<EnvMhaHashEntry_t*>* Coord2StateIDHashTable;

    //vector that maps from stateID to coords
    std::vector<EnvMhaHashEntry_t*> StateID2CoordTable;

    //any additional variables
} EnvironmentMha2D_t;
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

  virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV,
                        std::vector<int>* CostV) override;

  /**
   * \brief initialization environment from file (see .cfg files for examples)
   */
  virtual bool InitializeEnv(const char* sEnvFile) override;

  bool InitializeEnv(unsigned int width,
    unsigned int height,
    std::vector<sbpl_2Dpt_t>& footprint,
    double resolution,
    double nominalvel_mpersecs,
    double timetoturn45degsinplace_secs,
    unsigned char obsthresh,
    const char* motionPrimitivesFile);
  /**
   * \brief initialization of MDP data structure
   */
  virtual bool InitializeMDPCfg(MDPConfig* MDPCfg) override;

  bool IsValidCell(int X, int Y);

  bool IsWithinMapCell(int X, int Y);

  bool IsValidConfiguration(int X, int Y, int Theta);

  /**
   * \brief prints environment config file
   */
  virtual void PrintEnv_Config(FILE* fOut) override;

  /**
   * \brief prints the state variables for a state with stateID
   */
  virtual void PrintState(int stateID, bool bVerbose,
                          FILE* fOut = NULL) override;

  virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state) override;

  virtual void SetAllPreds(CMDPSTATE* state) override;

  // returns the stateid if success, and -1 otherwise
  int SetGoal(double x_m, double y_m, double theta_rad);

  // returns the stateid if success, and -1 otherwise
  int SetStart(double x_m, double y_m, double theta_rad);

  virtual int SizeofCreatedEnv() override;

 protected:
  EnvMhaConfig_t MhaEnvCfg;
  EnvironmentMha_t EnvMha2D;
};
}
