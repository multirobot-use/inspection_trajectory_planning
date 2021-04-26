#include "mission_planner_durable_leader.hpp"

MissionPlannerDurableLeader::MissionPlannerDurableLeader(parameters params)
    : MissionPlanner(params){};
MissionPlannerDurableLeader::~MissionPlannerDurableLeader(){};

std::vector<state> MissionPlannerDurableLeader::initialTrajectoryToInspect(){
  std::vector<state> traj;
  state state_in_circle, section_point;
  int goal = 0;
  int j = 1; // Iteration of the section
  int total_section_steps;
  float angle, pass_angle;
  static bool clockwise, initial_point_of_section = true;
  state_in_circle.pos = pointOnCircle(states_[param_.drone_id].pos);
  traj.push_back(state_in_circle);
  Eigen::Vector3d vel_unitary(0,0,0);

  for(int i = 1; i < param_.horizon_length; i++){

    // Temporary fix: if we do not consider the rest state, all the sections will go always in the same direction
    if (initial_point_of_section){
      vel_unitary     = (goals_[goal].pos - state_in_circle.pos)/(goals_[goal].pos - state_in_circle.pos).norm(); 
      clockwise       = isClockWise(vel_unitary, state_in_circle);
      pass_angle      = getPassAngle(state_in_circle, goals_[goal]);
      
      total_section_steps = getSectionSteps(state_in_circle, goals_[goal], clockwise);

      initial_point_of_section = false;
    }

    section_point       = getSectionPoint(state_in_circle, clockwise, pass_angle, total_section_steps, j);
    state_in_circle.pos = pointOnCircle(section_point.pos); // Maybe it is superfluous
    traj.push_back(state_in_circle);

    j++;

    if((state_in_circle.pos - goals_[goal].pos).norm() < 0.5){
      goal++;
      j = 0;
      initial_point_of_section = true;   // Check the next waypoint
    }

    if(goal == goals_.size()){

      while(i < (param_.horizon_length - 1)) {
        traj.push_back(state_in_circle);
        i++;}

      break;
    }
  }
  
  return traj;
}

state MissionPlannerDurableLeader::getSectionPoint(const state &_state, const bool &_clockwise, const float &_angle_pass, const int &_total_section_steps, const int &_iteration){
  state section_point;
  float current_angle = getAngle(_state);

  if (_clockwise)   current_angle = current_angle - _angle_pass;
  else              current_angle = current_angle + _angle_pass;

  // The angle is currently defined between [-M_PI, M_PI]
  if (current_angle < -M_PI)    current_angle = current_angle + 2*M_PI;
  if (current_angle > M_PI)     current_angle = current_angle - 2*M_PI;

  section_point.pos(0) = point_to_inspect_(0) + distance_to_inspect_point_*cos(current_angle);
  section_point.pos(1) = point_to_inspect_(1) + distance_to_inspect_point_*sin(current_angle);
  section_point.pos(2) = _state.pos(2)        + (goals_[0].pos(2) - _state.pos(2))*(_iteration/_total_section_steps);     // Check z

  return section_point;

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

float MissionPlannerDurableLeader::getPassAngle(const state &_initial_point, const state &_final_point){

  // Z variation between points increases total_linear_distance. It's important to take it into account
  float Z_diff = _final_point.pos(2) - _initial_point.pos(2);
  float total_linear_distance = sqrt(pow(distance_to_inspect_point_*2*M_PI,2) + pow(Z_diff,2)); // sqrt from math.h
  float step_linear_distance  = param_.vel_max*param_.step_size;

  float pass_angle = (step_linear_distance/total_linear_distance)*2*M_PI;

  std::cout << "Pass angle: " << std::to_string(pass_angle) << std::endl;

  return pass_angle;
}

// Need to know for the z increase in each section point
int MissionPlannerDurableLeader::getSectionSteps(const state &_initial_point, const state &_final_point, const bool &_clockwise){
  int total_steps = 0;

  float angle_pass      = getPassAngle(_initial_point, _final_point);
  float initial_angle   = getAngle(_initial_point);
  float final_angle     = getAngle(_final_point);

  float current_angle = initial_angle;

  while(1){

    if (_clockwise)   current_angle = current_angle - angle_pass;
    else              current_angle = current_angle + angle_pass;

    total_steps++;

    // It is easier to do these operations if they are done before the saturation/adjust of the angle
    if (_clockwise){
      if ((current_angle < final_angle) && (current_angle > (final_angle - angle_pass)))        return total_steps;
    }
    else if ((current_angle > final_angle) && (current_angle < (final_angle + angle_pass)))     return total_steps;

    // The angle is currently defined between [-M_PI, M_PI]
    if (current_angle < -M_PI)    current_angle = current_angle + 2*M_PI;
    if (current_angle > M_PI)     current_angle = current_angle - 2*M_PI;

  }

}

bool MissionPlannerDurableLeader::isClockWise(const Eigen::Vector3d &_vector, const state &_state){
  float angle = getAngle(_state);
  int quadrant;

  if ((angle >= -M_PI)   && (angle < -M_PI/2))     quadrant = 3;
  if ((angle >= -M_PI/2) && (angle < 0))           quadrant = 4;
  if ((angle >= 0)       && (angle < M_PI/2))      quadrant = 1;
  if ((angle >= M_PI/2)  && (angle <= M_PI))       quadrant = 2;

  // std::cout << std::endl << "QUADRANT: " << std::to_string(quadrant) << std::endl;
  // std::cout << std::endl << "ANGLE (rad): " << angle << std::endl;

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