#pragma once

#include "ros/ros.h"
#include <trajectory_planner_types.hpp>

struct inspection_params {
  float inspection_dist = 1;  // meters
  int leader_id = 1;
  float inc_distance = 0.1;
  float inc_angle = 0.01;
};

inline Eigen::Vector3d pointOnSphere(const Eigen::Vector3d point,
                                     const Eigen::Vector3d inspection_point,
                                     const float R) {
  return (R * (point - inspection_point) / ((point - inspection_point).norm()) +
          inspection_point);
}

inline Eigen::Vector3d transformToPolar(
    const Eigen::Vector3d &pose_xyz, const Eigen::Vector3d &_inspection_point) {
  Eigen::Vector3d pose_polar;
  float aux, aux2;

  pose_polar(0) = sqrt(pow(pose_xyz(0) - _inspection_point(0), 2) +
                       pow(pose_xyz(1) - _inspection_point(1), 2));  // r
  pose_polar(1) = atan2(pose_xyz(1) - _inspection_point(1),
                        pose_xyz(0) - _inspection_point(0));  // theta
  
  if (pose_polar(1) < 0) pose_polar(1) = pose_polar(1) + 2 * M_PI;
  if (pose_polar(1) > 2 * M_PI) {
    aux = pose_polar(1) / (2 * M_PI);
    aux2 = floor(aux);
    pose_polar(1) = (aux - aux2) * (2 * M_PI);
  }
  pose_polar(2) = pose_xyz(2);  // z
  return pose_polar;
}
// From -pi to pi
inline float getAngle(const Eigen::Vector3d _state,
                      const Eigen::Vector3d _inspection_point) {
  return atan2(_state(1) - _inspection_point(1),
               _state(0) - _inspection_point(0));
}

// From 0 to 2*pi
inline float getNormalizedAngle(const Eigen::Vector3d _state,
                                const Eigen::Vector3d _inspection_point) {
  float angle = getAngle(_state, _inspection_point);
  if (angle < 0)
    return (angle + 2 * M_PI);
  else
    return angle;
}

inline Eigen::Vector3d rotateEig(const Eigen::Vector3d &eigen_to_rotate,
                                 const float angle) {
  Eigen::Quaterniond rotation = trajectory_planner::eulerToQuat(0, 0, angle);
  Eigen::Matrix3d rotMat = rotation.toRotationMatrix();
  return rotMat * eigen_to_rotate;
}
