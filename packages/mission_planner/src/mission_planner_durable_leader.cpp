#include "mission_planner_durable_leader.hpp"

MissionPlannerDurableLeader::MissionPlannerDurableLeader(parameters params)
    : MissionPlanner(params){};
MissionPlannerDurableLeader::~MissionPlannerDurableLeader(){};

std::vector<state> MissionPlannerDurableLeader::initialTrajectoryToInspect(){
  std::vector<state> traj;
  state state_in_circle;
  static int goal = 0;
  float angle;
  bool clockwise;
  state_in_circle.pos = pointOnCircle(states_[param_.drone_id].pos);
  traj.push_back(state_in_circle);
  Eigen::Vector3d vel_unitary(0,0,0);
  // Eigen::Vector3d vel_goal2goal(0,0,0);

  for(int i = 1; i < param_.horizon_length; i++){
    vel_unitary         = (goals_[goal].pos-state_in_circle.pos)/(goals_[goal].pos-state_in_circle.pos).norm(); 

    state_in_circle.pos = pointOnCircle(state_in_circle.pos + vel_unitary*param_.vel_max*param_.step_size);
    traj.push_back(state_in_circle);

    if (i == 1){
      angle     = getAngle(state_in_circle);
      clockwise = isClockWise(vel_unitary, angle);

      if (clockwise)  std::cout << std::endl << std::endl << std::endl << "-------- The path between the current point and the goal point " << std::to_string(goal) << " is clockwise" << std::endl;
      else            std::cout << std::endl << std::endl << std::endl << "-------- The path between the current point and the goal point " << std::to_string(goal) << " is anticlockwise" << std::endl;
    }

    if((state_in_circle.pos-goals_[goal].pos).norm() < 0.5){goal++;}

    if(goal == goals_.size()){

      while(i < (param_.horizon_length - 1)) {
        traj.push_back(state_in_circle);
        i++;}

      break;
    }
  }
  // if (goal < goals_.size()-1){
  //   vel_goal2goal         = (goals_[goal + 1].pos - goals_[goal].pos)/(goals_[goal + 1].pos - goals_[goal].pos).norm();

  //   angle     = getAngle(goals_[goal]);
  //   clockwise = isClockWise(vel_goal2goal, angle);

  //   if (clockwise)  std::cout << std::endl << std::endl << std::endl << "-------- The path between the current point and the goal point " << std::to_string(goal) << " is clockwise" << std::endl;
  //   else            std::cout << std::endl << std::endl << std::endl << "-------- The path between the current point and the goal point " << std::to_string(goal) << " is anticlockwise" << std::endl;
  // }
  
  // std::cout<<traj.size()<<std::endl;
  return traj;
}

void MissionPlannerDurableLeader::appendGoal(const state &_new_goal) {
  state goal;

  Eigen::Vector3d goal_vector = _new_goal.pos;

  goal.pos = pointOnCircle(goal_vector);

  goals_.push_back(goal);
}

float MissionPlannerDurableLeader::getAngle(const state &_state){
  float _angle = atan2(_state.pos(1) - point_to_inspect_(1), _state.pos(0) - point_to_inspect_(0));

  return _angle;
}

bool MissionPlannerDurableLeader::isClockWise(const Eigen::Vector3d &_vector, float angle){
  int quadrant;

  if ((angle >= -M_PI)   && (angle < -M_PI/2))     quadrant = 3;
  if ((angle >= -M_PI/2) && (angle < 0))           quadrant = 4;
  if ((angle >= 0)       && (angle < M_PI/2))      quadrant = 1;
  if ((angle >= M_PI/2)  && (angle <= M_PI))       quadrant = 2;

  std::cout << std::endl << "QUADRANT: " << std::to_string(quadrant) << std::endl;
  std::cout << std::endl << "ANGLE (rad): " << angle << std::endl;

  // true if clockwise, false if anticlockwise
  switch (quadrant){

    case 1:
      if ((_vector(0) > 0) && (_vector(1) < 0))   return true;
      else                                        return false;
      break;

    case 2:
      if ((_vector(0) > 0) && (_vector(1) > 0))   return true;
      else                                        return false;
      break;

    case 3:
      if ((_vector(0) < 0) && (_vector(1) > 0))   return true;
      else                                        return false;
      break;

    case 4:
      if ((_vector(0) < 0) && (_vector(1) < 0))   return true;
      else                                        return false;
      break;

    default:
      break;
  }
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