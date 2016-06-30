#include "mha_global_planner/mha_environment.h"

namespace mha_global_planner {

MhaEnvironment::MhaEnvironment() {}

int MhaEnvironment::GetFromToHeuristic(int FromStateID, int ToStateID) {}

int MhaEnvironment::GetGoalHeuristic(int stateID) {}

int MhaEnvironment::GetStartHeuristic(int stateID) {}

void MhaEnvironment::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV,
                              std::vector<int>* CostV) {}

void MhaEnvironment::GetPreds(int TargetStateID, std::vector<int>* PredIDV,
                              std::vector<int>* CostV);

bool MhaEnvironment::InitializeEnv(const char* sEnvFile) {}

bool MhaEnvironment::InitializeEnv(unsigned int width,
    unsigned int height,
    std::vector<sbpl_2Dpt_t>& footprint,
    double resolution,
    double nominalvel_mpersecs,
    double timetoturn45degsinplace_secs,
    unsigned char obsthresh,
    const char* motionPrimitivesFile){
  MhaEnvCfg.EnvWidth_c = width;
  MhaEnvCfg.EnvHeight_c = height;
  MhaEnvCfg.obsthresh = obsthresh;
  MhaEnvCfg.cellsize_m = cellsize_m;
  MhaEnvCfg.StartTheta_rad = 0;
  MhaEnvCfg.EndTheta_rad = 0;
  MhaEnvCfg.StartX_c = 0;
  MhaEnvCfg.StartY_c = 0;
  MhaEnvCfg.EndX_c = 0;
  MhaEnvCfg.EndY_c = 0;

  for (int y = 0; y < EnvNAV2DCfg.EnvHeight_c; y++)
  {
    for (int x = 0; x < EnvNAV2DCfg.EnvWidth_c; x++)
    {
      EnvNAV2DCfg.Grid2D[x][y] = 0;
    }
  }

}

bool MhaEnvironment::InitializeMDPCfg(MDPConfig* MDPCfg) {}

void MhaEnvironment::PrintEnv_Config(FILE* fOut) {}

void MhaEnvironment::PrintState(int stateID, bool bVerbose, FILE* fOut = NULL) {
}

void MhaEnvironment::SetAllActionsandAllOutcomes(CMDPSTATE* state) {}

void MhaEnvironment::SetAllPreds(CMDPSTATE* state) {}

int MhaEnvironment::SetGoal(double x_m, double y_m, double theta_rad) {
  // first descretize the x and y coordinates into cell-size coordinates
  int x = CONTXY2DISC(x_m, MhaEnvCfg.cellsize_m);
  int y = CONTXY2DISC(y_m, MhaEnvCfg.cellsize_m);
  int theta = ContTheta2DiscNew(theta_rad);

  if (!IsWithinMapCell(x, y)) {
    ROS_ERROR(
        "ERROR: trying to set a start cell %d %d that is outside of map\n", x,
        y);
    return -1;
  }

  ROS_INFO("env: setting start to %.3f %.3f %.3f (%d %d %d)\n", x_m, y_m,
           theta_rad, x, y, theta);

  if (!IsValidConfiguration(x, y, theta)) {
    ROS_INFO("WARNING: start configuration %d %d %d is invalid\n", x, y, theta);
  }

  MhaEnvHashEntry_t* OutHashEntry = (this->*GetHashEntry)(x, y, theta);
  if (OutHashEntry == nullptr) {
    // have to create a new entry
    OutHashEntry = (this->*CreateNewHashEntry)(x, y, theta);
  }

  // need to recompute start heuristics?
  if (MhaEnv.startstateid != OutHashEntry->stateID) {
    bNeedtoRecomputeStartHeuristics = true;
    // because termination condition can be not all states TODO - make it
    // dependent on term. condition
    bNeedtoRecomputeGoalHeuristics = true;
  }

  // set start
  MhaEnv.startstateid = OutHashEntry->stateID;
  MhaEnvCfg.StartX_c = x;
  MhaEnvCfg.StartY_c = y;
  MhaEnvCfg.StartTheta = theta;

  return MhaEnv.startstateid;
}

bool MhaEnvironment::IsValidCell(int X, int Y) {
  return (X >= 0 && X < MhaEnvCfg.EnvWidth_c && Y >= 0 &&
          Y < MhaEnvCfg.EnvHeight_c &&
          MhaEnvCfg.Grid2D[X][Y] < MhaEnvCfg.obsthresh);
}

bool MhaEnvironment::IsWithinMapCell(int X, int Y) {
  return (X >= 0 && X < MhaEnvCfg.EnvWidth_c && Y >= 0 &&
          Y < MhaEnvCfg.EnvHeight_c);
}

bool MhaEnvironment::IsValidConfiguration(int X, int Y, int Theta) {
  std::vector<sbpl_2Dcell_t> footprint;
  sbpl_xy_theta_pt_t pose;

  // compute continuous pose
  pose.x = DISCXY2CONT(X, MhaEnvCfg.cellsize_m);
  pose.y = DISCXY2CONT(Y, MhaEnvCfg.cellsize_m);
  pose.theta = DiscTheta2ContNew(Theta);

  // compute footprint cells
  get_2d_footprint_cells(MhaEnvCfg.FootprintPolygon, &footprint, pose,
                         MhaEnvCfg.cellsize_m);

  // iterate over all footprint cells
  for (int find = 0; find < (int)footprint.size(); find++) {
    int x = footprint.at(find).x;
    int y = footprint.at(find).y;

    if (x < 0 || x >= MhaEnvCfg.EnvWidth_c || y < 0 ||
        y >= MhaEnvCfg.EnvHeight_c ||
        MhaEnvCfg.Grid2D[x][y] >= MhaEnvCfg.obsthresh) {
      return false;
    }
  }

  return true;
}

int MhaEnvironment::SetStart(double x_m, double y_m, double theta_rad) {}

int MhaEnvironment::SizeofCreatedEnv() {}
};
}
