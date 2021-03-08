#pragma once

#include <Eigen/StdVector>
#include <map>

#include "mission_planner_types.hpp"

class MissionPlanner {
 private:
  std::map<int, state> states_;
  /* data */
 public:
  MissionPlanner(/* args */);
  ~MissionPlanner();
};