#include "mission_planner_durable_leader.hpp"

MissionPlannerDurableLeader::MissionPlannerDurableLeader(parameters params)
    : MissionPlanner(params){};
MissionPlannerDurableLeader::~MissionPlannerDurableLeader(){};

std::vector<state> MissionPlannerDurableLeader::initialTrajectoryToInspect(){
  // TODO
  std::vector<state> traj;
  // int goal = 0;
  // Eigen::Vector3d pose(states_[param_.drone_id].pos(0), states_[param_.drone_id].pos(1), states_[param_.drone_id].pos(2));
  // traj.push_back(std::move(pose));

  // for(int i = 1; i < param_.horizon_length-1; i++){
  //   pose = pose+vel*param_.vel_max*param_.step_size;
  //   traj.push_back(pointOnCircle(pose);

  //   if((pose-goals_(goal)).norm<0.2){goal++;}

  //   if(goal==goals_.size()-1) while(i < (param_.horizon_length - 1)) {traj.push_back(pose);}
  // }

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