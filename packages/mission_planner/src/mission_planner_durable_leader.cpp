#include "mission_planner_durable_leader.hpp"

MissionPlannerDurableLeader::MissionPlannerDurableLeader(parameters params)
    : MissionPlanner(params){};
MissionPlannerDurableLeader::~MissionPlannerDurableLeader(){};

std::vector<state> MissionPlannerDurableLeader::initialTrajectory(const state &_initial_pose){
  std::vector<state> traj;
  state state_point_on_circle;
  Eigen::Vector3d aux_point2, point_on_circle;

  state_point_on_circle.pos(0) = _initial_pose.pos(0);
  state_point_on_circle.pos(1) = _initial_pose.pos(1);
  state_point_on_circle.pos(2) = _initial_pose.pos(2);

  traj.push_back(state_point_on_circle);

  for(int i = 0; i < (goals_.size() - 1); i++){
    for (int j = 1; j < param_.horizon_length; j++){
      aux_point2(0)  = (goals_.at(i+1).pos(0) - goals_.at(i).pos(0))*((j-1)/(param_.horizon_length)) + goals_.at(i).pos(0);
      aux_point2(1)  = (goals_.at(i+1).pos(1) - goals_.at(i).pos(1))*((j-1)/(param_.horizon_length)) + goals_.at(i).pos(1);
      aux_point2(2)  = (goals_.at(i+1).pos(2) - goals_.at(i).pos(2))*((j-1)/(param_.horizon_length)) + goals_.at(i).pos(2);

      point_on_circle   = pointOnCircle(aux_point2, point_to_inspect_, distance_to_inspect_point_);

      state_point_on_circle.pos(0) = point_on_circle(0);
      state_point_on_circle.pos(1) = point_on_circle(1);
      state_point_on_circle.pos(2) = point_on_circle(2);

      traj.push_back(state_point_on_circle);
    }
  }

  return traj;
}

void MissionPlannerDurableLeader::appendGoal(const state &_new_goal) {
  state goal;
  goal.pos = pointOnCircle(_new_goal.pos,point_to_inspect_, distance_to_inspect_point_);
  goals_.push_back(goal);
}

bool MissionPlannerDurableLeader::checks(){
  if(!hasGoal()){
    std::cout<<"Mission Planner "<<param_.drone_id<<" does not have goals"<<std::endl;
    return false;
  }
  if(!hasPose()){
    std::cout<<"Mission Planner "<<param_.drone_id<<" does not have all poses"<<std::endl;
    return false;
  }
  // check waypoints to remove or not the waypoints to follow
  if(waypointReached(goals_[0],last_trajectory_.back()) && planner_state_ == PlannerStatus::REPLANNED){
    std::cout<<"Removed waypoint"<<std::endl;
    goals_.erase(goals_.begin());
    if(!hasGoal()) return false;
  }
  return true;
}
