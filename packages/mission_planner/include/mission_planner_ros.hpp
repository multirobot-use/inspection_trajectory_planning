#pragma once
#include "mission_planner.hpp"
#include "ros/ros.h"

class MissionPlannerRos {
 private:
  ros::NodeHandle nh_;

 public:
  MissionPlannerRos(ros::NodeHandle _nh);
  ~MissionPlannerRos();
};
