#pragma once

#include <Eigen/StdVector>
#include <map>

#include "mission_planner_types.hpp"
enum PlannerStatus { FIRST_PLAN = 0, START_REPLANNING = 1, REPLANNED = 2 };

class MissionPlanner {
 private:
  const parameters param_;
  std::map<int, state> states_;
  std::vector<state> goals_;

 public:
  MissionPlanner(const parameters _param);
  ~MissionPlanner();
  void appendGoal(const state&);
  void clearGoals();
  void initialTrajectory();
  void optimalTrajectory();
  void plan();
};