#pragma once

#include <mission_planner_durable_follower.hpp>

MissionPlannerDurableFollower::MissionPlannerDurableFollower(parameters params) : MissionPlanner(params)
{}

MissionPlannerDurableFollower::~MissionPlannerDurableFollower(){}

void MissionPlannerDurableFollower::optimalTrajectory(
    const std::vector<state> &initial_trajectory){
    // TODO
}

Eigen::Vector3d MissionPlannerDurableFollower::pointOnCircle(
    const Eigen::Vector3d &_point){
  // TODO
  Eigen::Vector3d circle_point;
  circle_point = _point;

  return circle_point;
}

std::vector<state> MissionPlannerDurableFollower::initialTrajectory(
    // check if the distance is reached
      // parametricTrajectory();
    //else 
      // stdTrajectroy();
    // fake implementation
    const state &_state){
    std::vector<state> trajectory_to_optimize;
      
    // TODO
    return trajectory_to_optimize;
}