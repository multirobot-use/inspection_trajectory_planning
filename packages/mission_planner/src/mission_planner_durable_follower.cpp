#pragma once

#include <mission_planner_durable_follower.hpp>

MissionPlannerDurableFollower::MissionPlannerDurableFollower(parameters params) : MissionPlanner(params)
{}

MissionPlannerDurableFollower::~MissionPlannerDurableFollower(){}

void MissionPlannerDurableFollower::optimalTrajectory(
    const std::vector<state> &initial_trajectory){
    // TODO
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