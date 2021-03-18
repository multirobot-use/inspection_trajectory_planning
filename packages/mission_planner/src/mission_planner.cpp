#include "mission_planner.hpp"

MissionPlanner::MissionPlanner(const parameters _param) : param_(_param) {}

MissionPlanner::~MissionPlanner() {}

void MissionPlanner::appendGoal(const state &_new_goal) {
  goals_.push_back(_new_goal);

  // // Test
  // int goals_size = (goals_.size()-1);

  // for (int i=0; i < goals_size; i++)
  // {
  //   ROS_INFO("Already have point %d position x:   %f", i, goals_[i].pos[0]);
  // }
  // ROS_INFO("Receiving now point %d position x:   %f", goals_size, goals_[goals_size].pos[0]);
}

void MissionPlanner::clearGoals() { goals_.clear(); }

void MissionPlanner::plan() {
  // calculate initial trajectory
  // calculate optimal trajectory
}
