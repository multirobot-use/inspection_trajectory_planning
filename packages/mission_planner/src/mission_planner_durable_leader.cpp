#include "mission_planner_durable_leader.hpp"

MissionPlannerDurableLeader::MissionPlannerDurableLeader(parameters params)
    : MissionPlanner(params){};
MissionPlannerDurableLeader::~MissionPlannerDurableLeader(){};

std::vector<state> MissionPlannerDurableLeader::initialTrajectoryToInspect(){
  std::vector<state> traj;
  state state_in_circle;
  int goal = 0;
  state_in_circle.pos = pointOnCircle(states_[param_.drone_id].pos);
  traj.push_back(state_in_circle);
  Eigen::Vector3d vel_unitary(0,0,0);
  for(int i = 1; i < param_.horizon_length; i++){
    vel_unitary = (goals_[goal].pos-state_in_circle.pos)/(goals_[goal].pos-state_in_circle.pos).norm(); 
    state_in_circle.pos = pointOnCircle(state_in_circle.pos+vel_unitary*param_.vel_max*param_.step_size);
    traj.push_back(state_in_circle);
    if((state_in_circle.pos-goals_[goal].pos).norm()<0.5){goal++;}
    if(goal==goals_.size()){
      while(i < (param_.horizon_length - 1)) {
        traj.push_back(state_in_circle);
        i++;}
      break;
    }
  }
  std::cout<<traj.size()<<std::endl;
  return traj;
}

void MissionPlannerDurableLeader::appendGoal(const state &_new_goal) {
  state goal;

  Eigen::Vector3d goal_vector = _new_goal.pos;

  goal.pos = pointOnCircle(goal_vector);

  goals_.push_back(goal);
}

bool MissionPlannerDurableLeader::checks(){
  std::cout<<"checking... "<<std::endl;
  if(!hasGoal()){
    std::cout<<"Mission Planner "<<param_.drone_id<<" does not have goals"<<std::endl;
    return false;
  }
  if(!hasPose()){
    std::cout<<"Mission Planner "<<param_.drone_id<<" does not have all poses"<<std::endl;
    return false;
  }
  // check waypoints to remove or not the waypoints to follow
  if(waypointReached(goals_[0],states_[param_.drone_id])){
    std::cout<<"Removed waypoint"<<std::endl;
    goals_.erase(goals_.begin());
    if(!hasGoal()) return false;
  }
  return true;
}