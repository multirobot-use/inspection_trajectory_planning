#include "mission_planner_ros.hpp"

MissionPlannerRos::MissionPlannerRos(ros::NodeHandle _nh) : nh_(_nh) {
  // ros params
  safeGetParam(nh_, "horizon_length", param_.horizon_length);
  safeGetParam(nh_, "step_size", param_.step_size);
  // initialize mission planner
  mission_planner_ptr_ = std::make_unique<MissionPlanner>(param_);
}

MissionPlannerRos::~MissionPlannerRos() {}
