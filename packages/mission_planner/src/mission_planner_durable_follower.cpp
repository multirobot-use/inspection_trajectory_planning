#include <mission_planner_durable_follower.hpp>

MissionPlannerDurableFollower::MissionPlannerDurableFollower(parameters params) : MissionPlanner(params)
{}

MissionPlannerDurableFollower::~MissionPlannerDurableFollower(){}


bool MissionPlannerDurableFollower::checks(){
  if(!hasPose()){
    std::cout<<"Mission Planner "<<param_.drone_id<<" does not have all poses"<<std::endl;
    return false;
  }
  return true;
}
std::vector<state> MissionPlannerDurableFollower::initialTrajectory(const state &_state){
  const Eigen::Vector3d goal = rotateEig(states_[1].pos,formation_angle_);
  return pathFromPointToAnother(states_[param_.drone_id].pos, goal);
}

std::vector<state> MissionPlannerDurableFollower::initialTrajectoryToInspect(const state &initial_pose){
    state aux;
    std::vector<state> trajectory_to_optimize;
    Eigen::Quaterniond rotation = eulerToQuat(0,0,formation_angle_);
    Eigen::Matrix3d rotMat = rotation.toRotationMatrix();
    Eigen::Vector3d aux_point_to_inspect = point_to_inspect_;
    point_to_inspect_(2) = 0;
    for(auto const &pose : solved_trajectories_[param_.leader_id]){
      aux.pos = rotMat*(pose - point_to_inspect_);
      trajectory_to_optimize.push_back(std::move(aux));
    }
    return trajectory_to_optimize;
}