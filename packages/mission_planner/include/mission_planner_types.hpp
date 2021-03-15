#pragma once

#include "ros/ros.h"
struct parameters {
  float horizon_length = 0;  // number of steps
  int n_drones = 1;          // number of drones
  float step_size = 0.1;     // seconds
};

struct state {
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
};

template <typename T>
inline bool safeGetParam(ros::NodeHandle& _nh, std::string const& _param_name,
                         T& _param_value) {
  if (!_nh.getParam(_param_name, _param_value)) {
    ROS_ERROR("Failed to find parameter: %s",
              _nh.resolveName(_param_name, true).c_str());
    exit(1);
  }
  return true;
}