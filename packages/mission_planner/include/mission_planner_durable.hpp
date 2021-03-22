#pragma once
#include "mission_planner.hpp"
class MissionPlannerDurable : public MissionPlanner {
 public:
  MissionPlannerDurable(parameters params);
  ~MissionPlannerDurable();

 private:
  std::vector<state> initialTrajectory(const state &_state);
  void optimalTrajectory(const std::vector<state> &initial_trajectory);
};