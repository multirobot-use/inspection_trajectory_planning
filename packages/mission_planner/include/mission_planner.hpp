#pragma once

#include <Eigen/StdVector>
#include <map>

#include "mission_planner_types.hpp"

class MissionPlanner {
 private:
  const parameters param_;
  std::map<int, state> states_;
  /* data */
 public:
  MissionPlanner(const parameters _param);
  ~MissionPlanner();
};