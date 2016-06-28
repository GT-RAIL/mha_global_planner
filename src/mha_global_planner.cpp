#include "mha_global_planner/mha_global_planner.h"

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>

// register plugin
PLUGINLIB_EXPORT_CLASS(mha_global_planner::MhaGlobalPlanner,
                       nav_core::BaseGlobalPlanner)

namespace mha_global_planner {

class MhaStateQueryChange : public StateChangeQuery {
 public:
  MhaStateQueryChange(EnvironmentNAVXYTHETALAT* env,
                      std::vector<nav2dcell_t> const& changedcellsV)
      : env_(env), changedcellsV_(changedcellsV) {}

  // lazy init, because we do not always end up calling this method
  virtual std::vector<int> const* getPredecessors() const {
    if (predsOfChangedCells_.empty() && !changedcellsV_.empty())
      env_->GetPredsofChangedEdges(&changedcellsV_, &predsOfChangedCells_);
    return &predsOfChangedCells_;
  }

  // lazy init, because we do not always end up calling this method
  virtual std::vector<int> const* getSuccessors() const {
    if (succsOfChangedCells_.empty() && !changedcellsV_.empty())
      env_->GetSuccsofChangedEdges(&changedcellsV_, &succsOfChangedCells_);
    return &succsOfChangedCells_;
  }

  EnvironmentNAVXYTHETALAT* env_;
  std::vector<nav2dcell_t> const& changedcellsV_;
  mutable std::vector<int> predsOfChangedCells_;
  mutable std::vector<int> succsOfChangedCells_;
};

MhaGlobalPlanner::MhaGlobalPlanner() : initialized_(false) {}

MhaGlobalPlanner::MhaGlobalPlanner(std::string name,
                                   costmap_2d::Costmap2DROS* costmap_ros) {
  initialize(name, costmap_ros);
}

void MhaGlobalPlanner::initialize(std::string name,
                                  costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("This planner has already been initialized... doing nothing");
    return;
  }

  initialized_ = true;

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  ros::NodeHandle private_nh("~" + name);
  ros::NodeHandle nh;

  int lethal_obstacle;
  private_nh.param("primitive_filename", primitive_filename_, std::string(""));
  private_nh.param("allocated_time", allocated_time_, 10.0);
  private_nh.param("initial_epsilon", initial_epsilon_, 3.0);
  private_nh.param("force_scratch_limit", force_scratch_limit_, 500);
  private_nh.param("lethal_obstacle", lethal_obstacle, 20);

  lethal_obstacle_ = (unsigned char)lethal_obstacle;
  inscribed_inflated_obstacle_ = lethal_obstacle_ - 1;

  env_ = new EnvironmentNAVXYTHETALAT();

  obst_cost_thresh_ = costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE);

  // mha needs the footprint of the robot. We assume it is constant.
  std::vector<geometry_msgs::Point> footprint =
      costmap_ros_->getRobotFootprint();
  std::vector<sbpl_2Dpt_t> perimeterptsV;
  perimeterptsV.reserve(footprint.size());
  for (size_t ii(0); ii < footprint.size(); ++ii) {
    sbpl_2Dpt_t pt;
    pt.x = footprint[ii].x;
    pt.y = footprint[ii].y;
    perimeterptsV.push_back(pt);
  }

  bool ret;
  try {
    ret = env_->InitializeEnv(
        costmap_ros_->getCostmap()->getSizeInCellsX(),  // width
        costmap_ros_->getCostmap()->getSizeInCellsY(),  // height
        nullptr,  // mapdata, initialized to freespace
        0, 0, 0,  // start (x, y, theta)
        0, 0, 0,  // goal (x, y, theta)
        0, 0, 0,  // goal tolerance (ingored right now)
        perimeterptsV, costmap_ros_->getCostmap()->getResolution(),
        FORWARD_PLANNING_SPEED_, ROT_PLANNING_SPEED_, obst_cost_thresh_,
        primitive_filename_.c_str());
  } catch (SBPL_Exception e) {
    ROS_ERROR("SBPL encountered a fatal exception!");
    ret = false;
  }
  if (!ret) {
    ROS_ERROR("SBPL initialization failed!");
    exit(1);
  }

  for (ssize_t ix(0); ix < costmap_ros_->getCostmap()->getSizeInCellsX();
       ++ix) {
    for (ssize_t iy(0); iy < costmap_ros_->getCostmap()->getSizeInCellsY();
         ++iy) {
      env_->UpdateCost(
          ix, iy,
          costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix, iy)));
    }
  }

  mha_planner_ = new MHAPlanner(env_, anchor_heuristic_, heuristics_.data(),
                                heuristics_.size());

  plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
}

bool MhaGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    ROS_ERROR("Global planner is not initialized");
    return false;
  }

  plan.clear();

  ROS_INFO("[sbpl_mha_planner] getting start point (%g,%g) goal point (%g,%g)",
           start.pose.position.x, start.pose.position.y, goal.pose.position.x,
           goal.pose.position.y);
  double theta_start =
      2 * atan2(start.pose.orientation.z, start.pose.orientation.w);
  double theta_goal =
      2 * atan2(goal.pose.orientation.z, goal.pose.orientation.w);

  try {
    int ret = env_->SetStart(
        start.pose.position.x - costmap_ros_->getCostmap()->getOriginX(),
        start.pose.position.y - costmap_ros_->getCostmap()->getOriginY(),
        theta_start);
    if (ret < 0 || mha_planner_->set_start(ret) == 0) {
      ROS_ERROR("ERROR: failed to set start state\n");
      return false;
    }
  } catch (SBPL_Exception e) {
    ROS_ERROR(
        "SBPL encountered a fatal exception while setting the start state");
    return false;
  }

  try {
    int ret = env_->SetGoal(
        goal.pose.position.x - costmap_ros_->getCostmap()->getOriginX(),
        goal.pose.position.y - costmap_ros_->getCostmap()->getOriginY(),
        theta_goal);
    if (ret < 0 || mha_planner_->set_goal(ret) == 0) {
      ROS_ERROR("ERROR: failed to set goal state\n");
      return false;
    }
  } catch (SBPL_Exception e) {
    ROS_ERROR(
        "SBPL encountered a fatal exception while setting the goal state");
    return false;
  }

  int offOnCount = 0;
  int onOffCount = 0;
  int allCount = 0;
  std::vector<nav2dcell_t> changedcellsV;

  for (unsigned int ix = 0; ix < costmap_ros_->getCostmap()->getSizeInCellsX();
       ix++) {
    for (unsigned int iy = 0;
         iy < costmap_ros_->getCostmap()->getSizeInCellsY(); iy++) {
      unsigned char oldCost = env_->GetMapCost(ix, iy);
      unsigned char newCost =
          costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix, iy));

      if (oldCost == newCost) continue;

      allCount++;

      // first case - off cell goes on

      if ((oldCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) &&
           oldCost != costMapCostToSBPLCost(
                          costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) ||
           newCost == costMapCostToSBPLCost(
                          costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        offOnCount++;
      }

      if ((oldCost == costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) ||
           oldCost == costMapCostToSBPLCost(
                          costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) &&
          (newCost != costMapCostToSBPLCost(costmap_2d::LETHAL_OBSTACLE) &&
           newCost != costMapCostToSBPLCost(
                          costmap_2d::INSCRIBED_INFLATED_OBSTACLE))) {
        onOffCount++;
      }
      env_->UpdateCost(
          ix, iy,
          costMapCostToSBPLCost(costmap_ros_->getCostmap()->getCost(ix, iy)));

      nav2dcell_t nav2dcell;
      nav2dcell.x = ix;
      nav2dcell.y = iy;
      changedcellsV.push_back(nav2dcell);
    }
  }

  try {
    if (!changedcellsV.empty()) {
      StateChangeQuery* scq = new MhaStateQueryChange(env_, changedcellsV);
      mha_planner_->costs_changed(*scq);
      delete scq;
    }

    if (allCount > force_scratch_limit_)
      mha_planner_->force_planning_from_scratch();
  } catch (SBPL_Exception e) {
    ROS_ERROR("SBPL failed to update the costmap");
    return false;
  }

  // setting planner parameters
  ROS_DEBUG("allocated:%f, init eps:%f\n", allocated_time_, initial_epsilon_);
  mha_planner_->set_initialsolution_eps(initial_epsilon_);
  mha_planner_->set_search_mode(false);

  ROS_DEBUG("[sbpl_mha_planner] run planner");
  std::vector<int> solution_stateIDs;
  int solution_cost;
  try {
    int ret = mha_planner_->replan(allocated_time_, &solution_stateIDs,
                                   &solution_cost);
    if (ret)
      ROS_DEBUG("Solution is found\n");
    else {
      ROS_INFO("Solution not found\n");
      return false;
    }
  } catch (SBPL_Exception e) {
    ROS_ERROR("SBPL encountered a fatal exception while planning");
    return false;
  }

  ROS_DEBUG("size of solution=%d", (int)solution_stateIDs.size());

  std::vector<EnvNAVXYTHETALAT3Dpt_t> sbpl_path;
  try {
    env_->ConvertStateIDPathintoXYThetaPath(&solution_stateIDs, &sbpl_path);
  } catch (SBPL_Exception e) {
    ROS_ERROR(
        "SBPL encountered a fatal exception while reconstructing the path");
    return false;
  }
  ROS_DEBUG("Plan has %d points.\n", (int)sbpl_path.size());
  ros::Time plan_time = ros::Time::now();

  // create a message for the plan
  nav_msgs::Path gui_path;
  gui_path.poses.resize(sbpl_path.size());
  gui_path.header.frame_id = costmap_ros_->getGlobalFrameID();
  gui_path.header.stamp = plan_time;
  for (unsigned int i = 0; i < sbpl_path.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = plan_time;
    pose.header.frame_id = costmap_ros_->getGlobalFrameID();

    pose.pose.position.x =
        sbpl_path[i].x + costmap_ros_->getCostmap()->getOriginX();
    pose.pose.position.y =
        sbpl_path[i].y + costmap_ros_->getCostmap()->getOriginY();
    pose.pose.position.z = start.pose.position.z;

    tf::Quaternion temp;
    temp.setRPY(0, 0, sbpl_path[i].theta);
    pose.pose.orientation.x = temp.getX();
    pose.pose.orientation.y = temp.getY();
    pose.pose.orientation.z = temp.getZ();
    pose.pose.orientation.w = temp.getW();

    plan.push_back(pose);

    gui_path.poses[i].pose.position.x = plan[i].pose.position.x;
    gui_path.poses[i].pose.position.y = plan[i].pose.position.y;
    gui_path.poses[i].pose.position.z = plan[i].pose.position.z;
  }
  plan_pub_.publish(gui_path);

  return true;
}

// Taken from Sachin's sbpl_cart_planner
// This rescales the costmap according to a rosparam which sets the obstacle
// cost
unsigned char MhaGlobalPlanner::costMapCostToSBPLCost(unsigned char newcost) {
  if (newcost == costmap_2d::LETHAL_OBSTACLE) {
    return lethal_obstacle_;
  } else if (newcost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
    return inscribed_inflated_obstacle_;
  } else if (newcost == 0 || newcost == costmap_2d::NO_INFORMATION) {
    return 0;
  } else {
    return (unsigned char)(newcost / sbpl_cost_multiplier_ + 0.5);
  }
}
}
