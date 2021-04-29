#include "mission_planner_durable_leader.hpp"

MissionPlannerDurableLeader::MissionPlannerDurableLeader(parameters params)
    : MissionPlanner(params){};
MissionPlannerDurableLeader::~MissionPlannerDurableLeader(){};

std::vector<state> MissionPlannerDurableLeader::initialTrajectoryToInspect(const state &initial_pose){
  std::vector<state> traj;
  state state_in_circle;
  int goal = 0;
  // int total_section_steps;
  float vel_polar_angle;
  state_in_circle.pos = pointOnCircle(initial_pose.pos);
  traj.push_back(state_in_circle);
  Eigen::Vector3d vel_unitary(0,0,0);

  // Temporary fix: if we do not consider the rest state, all the sections will go always in the same direction
  vel_unitary     = (goals_[goal].pos - state_in_circle.pos)/(goals_[goal].pos - state_in_circle.pos).norm();

  // Not changing, at least for now
  if (PlannerStatus::FIRST_PLAN){
    clockwise = isClockWise(vel_unitary, state_in_circle);
  }

  vel_polar_angle = velmaxToPolar();

  for(int i = 1; i < param_.horizon_length; i++){

    // Temporary fix: if we do not consider the rest state, all the sections will go always in the same direction
    vel_unitary       = (goals_[goal].pos - state_in_circle.pos)/(goals_[goal].pos - state_in_circle.pos).norm(); 
    state_in_circle   = calcNextPoint(state_in_circle, clockwise, vel_polar_angle, goal, vel_unitary);
    traj.push_back(state_in_circle);

    if((state_in_circle.pos - goals_[goal].pos).norm() < 0.5){
      std::cout << std::endl << "Reached a goal during interpolation!" << std::endl;
      goal++;
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

state MissionPlannerDurableLeader::calcNextPoint(const state &_state, const bool &_clockwise, const float &_angle_pass, const int &_goal, const Eigen::Vector3d &_vel){
  state next_point;
  float current_angle = getNormalizedAngle(_state.pos, point_to_inspect_);

  if (_clockwise)   current_angle = current_angle - _angle_pass;
  else              current_angle = current_angle + _angle_pass;

  // The angle is currently defined between [-M_PI, M_PI]
  if (current_angle < 0)        current_angle = current_angle + 2*M_PI;

  next_point.pos(0) = point_to_inspect_(0) + distance_to_inspect_point_*cos(current_angle);
  next_point.pos(1) = point_to_inspect_(1) + distance_to_inspect_point_*sin(current_angle);
  next_point.pos(2) = _state.pos(2)        + _vel(2)*param_.step_size;     // Check z --> get it using vel_unitary or in function of next
  next_point.vel(0) = 0;
  next_point.vel(1) = 0;
  next_point.vel(2) = 0;

  return next_point;
}

void MissionPlannerDurableLeader::appendGoal(const state &_new_goal) {
  state goal;

  Eigen::Vector3d goal_vector = _new_goal.pos;

  goal.pos = pointOnCircle(goal_vector);

  goals_.push_back(goal);
}

float MissionPlannerDurableLeader::velmaxToPolar(){
  // Ang_vel = step_linear_distance / total_linear_distance
  return ((param_.vel_max_xy*param_.step_size)/(distance_to_inspect_point_*2*M_PI))*2*M_PI;
}

float MissionPlannerDurableLeader::getTotalAngleOfSection(const float &_initial_angle, const float &_final_angle, const bool &_clockwise){
  float total_angle, initial_angle, final_angle;

  if (_initial_angle < 0)         initial_angle  = _initial_angle  + 2*M_PI;
  else                            initial_angle  = _initial_angle;
  if (_final_angle < 0)           final_angle    = _final_angle    + 2*M_PI;
  else                            final_angle    = _final_angle;

  if (_clockwise){
    if (initial_angle < final_angle)    total_angle = initial_angle + (2*M_PI - final_angle);
    else                                total_angle = final_angle - initial_angle;
  }
  else{
    if (initial_angle > final_angle)    total_angle = final_angle + (2*M_PI - initial_angle);
    else                                total_angle = initial_angle - final_angle;
  }

  return total_angle;
}

// Need to know for the z increase in each section point
int MissionPlannerDurableLeader::getSectionSteps(const state &_initial_point, const state &_final_point, const bool &_clockwise){
  int total_steps = 0;

  float angle_pass      = velmaxToPolar();
  float initial_angle   = getAngle(_initial_point.pos, point_to_inspect_);
  float final_angle     = getAngle(_final_point.pos, point_to_inspect_);

  float current_angle = initial_angle;

  while(1){

    if (_clockwise)   current_angle = current_angle - angle_pass;
    else              current_angle = current_angle + angle_pass;

    total_steps++;

    // It is easier to do these operations if they are done before the saturation/adjust of the angle
    if (_clockwise){
      if ((current_angle < final_angle) && (current_angle > (final_angle - angle_pass)))        break;
    }
    else if ((current_angle > final_angle) && (current_angle < (final_angle + angle_pass)))     break;

    // The angle is currently defined between [-M_PI, M_PI]
    if (current_angle < -M_PI)    current_angle = current_angle + 2*M_PI;
    if (current_angle > M_PI)     current_angle = current_angle - 2*M_PI;

  }

  std::cout << "Total steps: " << std::to_string(total_steps) << std::endl;

  return total_steps;
}

bool MissionPlannerDurableLeader::isClockWise(const Eigen::Vector3d &_vector, const state &_state){
  float angle = getAngle(_state.pos, point_to_inspect_);
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