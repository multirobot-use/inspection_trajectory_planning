#include "mission_planner_durable_leader.hpp"

MissionPlannerDurableLeader::MissionPlannerDurableLeader(parameters params)
    : MissionPlanner(params){};
MissionPlannerDurableLeader::~MissionPlannerDurableLeader(){};


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

  std::vector<state> MissionPlannerDurableLeader::initialTrajectoryToInspect(){
    
  }
