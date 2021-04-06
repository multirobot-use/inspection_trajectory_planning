#pragma once
#include "mission_planner.hpp"
class MissionPlannerDurableLeader : public MissionPlanner {
 public:
  MissionPlannerDurableLeader(parameters params);
  ~MissionPlannerDurableLeader();

 private:
  std::vector<state> initialTrajectory(const state &_state);
  void optimalTrajectory(const std::vector<state> &initial_trajectory);
};