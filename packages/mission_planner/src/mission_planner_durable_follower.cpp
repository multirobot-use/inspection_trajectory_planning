#include <mission_planner_durable_follower.hpp>

MissionPlannerDurableFollower::MissionPlannerDurableFollower(parameters params) : MissionPlanner(params)
{}

MissionPlannerDurableFollower::~MissionPlannerDurableFollower(){}

void MissionPlannerDurableFollower::optimalTrajectory(
    const std::vector<state> &initial_trajectory){
    // TODO
}

std::vector<state> MissionPlannerDurableFollower::initialTrajectory(
    const state &_state){
    state aux;
    std::vector<state> trajectory_to_optimize;
    Eigen::Quaterniond rotation = eulerToQuat(0,0,formation_angle_);
    Eigen::Matrix3d rotMat = rotation.toRotationMatrix();
    for(auto const &pose : solved_trajectories_[param_.leader_id]){
      aux.pos = rotMat*(pose - point_to_inspect_);
      trajectory_to_optimize.push_back(std::move(aux));
    }
    return trajectory_to_optimize;
}