#pragma once
#include "mission_planner.hpp"
#include "ros/ros.h"

class MissionPlannerRos {
 private:
  ros::NodeHandle nh_;
  parameters param_;
  std::unique_ptr<MissionPlanner> mission_planner_ptr_;

 public:
  MissionPlannerRos(ros::NodeHandle _nh);
  ~MissionPlannerRos();
};
