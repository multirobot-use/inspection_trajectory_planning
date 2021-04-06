#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <acado/acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <map>
#include "mission_planner_types.hpp"

//! MissionPlanner class
/*!
 * Abstract base class for mission planner. It cannot be instatiated
 */

enum PlannerStatus { FIRST_PLAN = 0, REPLANNED = 2 };

class MissionPlanner {
 public:
  std::vector<state> last_trajectory_;
  std::map<int, state> states_;
  MissionPlanner(const parameters _param);
  virtual ~MissionPlanner();
  void appendGoal(const state &);
  void clearGoals();
  void setPointToInspect(const Eigen::Vector3d &);
  void plan();

  
 protected:
  const parameters param_;
  const ACADO::Grid my_grid_;
  std::vector<state> goals_;
  Eigen::Vector3d point_to_inspect_;


 private:

  int planner_state_ = PlannerStatus::FIRST_PLAN;
  void checkWaypoints();
  virtual std::vector<state> initialTrajectory(const state &_initial_pose) = 0;
  virtual void optimalTrajectory(const std::vector<state> &_initial_traj) = 0;
};