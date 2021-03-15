#include "mission_planner.hpp"

MissionPlanner::MissionPlanner(const parameters _param) : param_(_param) {}

MissionPlanner::~MissionPlanner() {}

void MissionPlanner::appendGoal(const state &_new_goal) {
  goals_.push_back(_new_goal);
}

void MissionPlanner::clearGoals() { goals_.clear(); }

void MissionPlanner::plan() {
  // calculate initial trajectory
  // calculate optimal trajectory
}
