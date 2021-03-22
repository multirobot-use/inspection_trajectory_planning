#include "mission_planner.hpp"

MissionPlanner::MissionPlanner(const parameters _param)
    : param_(_param),
      my_grid_(0.0, (param_.horizon_length - 1) * param_.step_size,
               param_.horizon_length) {}

MissionPlanner::~MissionPlanner() {}

void MissionPlanner::appendGoal(const state &_new_goal) {
  goals_.push_back(_new_goal);
}

void MissionPlanner::clearGoals() { goals_.clear(); }

void MissionPlanner::plan() {
  state initial_pose;
  if (planner_state_ == PlannerStatus::FIRST_PLAN) {
    initial_pose = states_[param_.drone_id];
  } else if (planner_state_ == PlannerStatus::REPLANNED) {
    initial_pose = last_trajectory_[param_.planning_rate];
  }
  // check waypoints to remove or not the waypoints to follow
  checkWaypoints();
  // calculate initial trajectory
  std::vector<state> initial_traj = initialTrajectory(initial_pose);
  // calculate optimal trajectory
  optimalTrajectory(initial_traj);
}

void MissionPlanner::checkWaypoints(){};
