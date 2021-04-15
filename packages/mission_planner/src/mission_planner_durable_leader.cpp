#include "mission_planner_durable_leader.hpp"

MissionPlannerDurableLeader::MissionPlannerDurableLeader(parameters params)
    : MissionPlanner(params){};
MissionPlannerDurableLeader::~MissionPlannerDurableLeader(){};

std::vector<state> MissionPlannerDurableLeader::initialTrajectoryToInspect(){
  std::vector<state> traj;
  state point_on_circle;
  Eigen::Vector3d traj_point, vel, previous_point, next_point;
  int i; // goals
  int j; // points

  for (i = 0; i < (goals_.size()-1); i++){
    if (i == 0){
      previous_point(0) = states_[param_.drone_id].pos(0);
      previous_point(1) = states_[param_.drone_id].pos(1);
      previous_point(2) = states_[param_.drone_id].pos(2);
    }
    else{
      previous_point(0) = goals_.at(i-1).pos(0);
      previous_point(1) = goals_.at(i-1).pos(1);
      previous_point(2) = goals_.at(i-1).pos(2);
    }

    next_point(0) = goals_.at(i).pos(0);
    next_point(1) = goals_.at(i).pos(1);
    next_point(2) = goals_.at(i).pos(2);

    vel = (next_point - previous_point)/(next_point - previous_point).norm();

    for (j = 0; j < param_.horizon_length; j++){
      traj_point = previous_point +  j*vel*param_.vel_max*param_.step_size;
    
      point_on_circle.pos = pointOnCircle(traj_point);
      // point_on_circle.vel = vel;
      traj.push_back(std::move(point_on_circle));
    }
  }

  return traj;
}

void MissionPlannerDurableLeader::appendGoal(const state &_new_goal) {
  state goal;

  Eigen::Vector3d goal_vector = _new_goal.pos;

  goal.pos = pointOnCircle(goal_vector);

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