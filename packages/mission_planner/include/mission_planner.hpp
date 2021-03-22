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

enum PlannerStatus { FIRST_PLAN = 0, START_REPLANNING = 1, REPLANNED = 2 };

class MissionPlanner {
 public:
  MissionPlanner(const parameters _param);
  virtual ~MissionPlanner();
  void appendGoal(const state &);
  void clearGoals();
  void plan();
  
 protected:
  const parameters param_;
  const ACADO::Grid my_grid_;
  std::vector<state> last_trajectory_;

 private:

  std::map<int, state> states_;
  std::vector<state> goals_;
  int planner_state_ = 0;
  void checkWaypoints();
  virtual std::vector<state> initialTrajectory(const state &_initial_pose) = 0;
  virtual void optimalTrajectory(const std::vector<state> &_initial_traj) = 0;
};