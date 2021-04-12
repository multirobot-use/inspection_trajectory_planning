#include "mission_planner.hpp"

MissionPlanner::MissionPlanner(const parameters _param)
    : param_(_param),
      last_trajectory_(_param.horizon_length),
      my_grid_(0.0, (param_.horizon_length - 1) * param_.step_size,
               param_.horizon_length) {}

MissionPlanner::~MissionPlanner() {}

void MissionPlanner::plan() {
  
  if(!hasGoal() || !hasPose()) {
    std::cout<<"Mission planner is not ready"<<std::endl;
    return;
  }

  state initial_pose = states_[param_.drone_id];

  // if (planner_state_ == PlannerStatus::FIRST_PLAN) {
  //   initial_pose = states_[param_.drone_id];
  // } else if (planner_state_ == PlannerStatus::REPLANNED) {
  //   initial_pose = last_trajectory_[param_.planning_rate];
  // }

  // check waypoints to remove or not the waypoints to follow
  if(waypointReached(goals_[0],last_trajectory_.back()) && planner_state_ == PlannerStatus::REPLANNED){
    std::cout<<"Removed waypoint"<<std::endl;
    goals_.erase(goals_.begin());
    if(goals_.empty()) return;
  } 
  // calculate initial trajectory
  std::vector<state> initial_traj = initialTrajectory(initial_pose);
  // calculate optimal trajectory
  optimalTrajectory(initial_traj);
  // calculate orientation
  initialOrientation(last_trajectory_);

  planner_state_ = PlannerStatus::REPLANNED;
}

void MissionPlanner::initialOrientation(std::vector<state> &traj){
  Eigen::Vector3d aux;
  for(auto &point : traj){
      aux = point_to_inspect_ - point.pos;
      point.orientation = eulerToQuat(0,0,atan2(aux(1), aux(0)));
  }
}

void MissionPlanner::optimalOrientation(const std::vector<state> &traj_to_optimize){
  // // DifferentialState heading, pitch, v_heading, v_pitch;
  // DifferentialState heading, v_heading;
  // // Control a_heading, a_pitch;
  // Control a_heading;

  // float a_limit     = 0.5;
  // float v_limit     = 0.5;
  // float pitch_limit = 1.57;
  // DifferentialEquation model;

  // model << dot(heading) == v_heading;
  // model << dot(v_heading) == a_heading;
  // // model << dot(pitch) == v_pitch;
  // // model << dot(v_pitch) == a_pitch;

  // OCP ocp(my_grid_);  
  // ocp.subjectTo(model);
  // ocp.subjectTo(-a_limit <= a_heading <= a_limit);
  // // ocp.subjectTo(-a_limit <= a_pitch <= a_limit);
  // ocp.subjectTo(-v_limit <= v_heading <= v_limit);
  // // ocp.subjectTo(-v_limit <= v_pitch <= v_limit);
  // // ocp.subjectTo(-pitch_limit <= pitch <= pitch_limit);

  // VariablesGrid reference_trajectory(2, my_grid_); // 2 for pitch
  // DVector       reference_point(2); // 2 for pitch
  // reference_trajectory.setVector(0, reference_point);  // TODO: check indexing
  // for (int k = 0; k < TIME_HORIZON; k++) {
  //   reference_point(0) = mrs_lib::geometry::radians::unwrap(_desired_trajectory[k].heading, reference_point(0));
  //   reference_point(1) = 0;
  //   reference_trajectory.setVector(k, reference_point);  // TODO: check indexing
  // }

  // // set initial guess so that it meats the constraints
  // // _initial_guess[0].v_heading = _initial_guess[0].v_heading < -v_limit ? -v_limit : _initial_guess[0].v_heading;
  // // _initial_guess[0].v_pitch   = _initial_guess[0].v_pitch > v_limit ? v_limit : _initial_guess[0].v_pitch;
  // // _initial_guess[0].a_heading = _initial_guess[0].a_heading < -a_limit ? -a_limit : _initial_guess[0].a_heading;
  // // _initial_guess[0].a_pitch   = _initial_guess[0].a_pitch > a_limit ? a_limit : _initial_guess[0].a_pitch;


  // ocp.subjectTo(AT_START, heading == _initial_guess[0].heading);
  // ocp.subjectTo(AT_START, pitch == _initial_guess[0].pitch);
  // ocp.subjectTo(AT_START, v_heading == _initial_guess[0].v_heading);
  // ocp.subjectTo(AT_START, v_pitch == _initial_guess[0].v_pitch);
  // ocp.subjectTo(AT_START, a_heading == _initial_guess[0].a_heading);
  // ocp.subjectTo(AT_START, a_pitch == _initial_guess[0].a_pitch);

  // Function rf;

  // rf << heading << a_heading;
  // // rf << pitch;

  // DMatrix S(2, 2);
  // // DMatrix S(2, 2);

  // S.setIdentity();
  // S(0, 0) = 1.0;
  // S(1, 1) = 1.0;

  // ocp.minimizeLSQ(S, rf, reference_trajectory);

 
  // OptimizationAlgorithm solver_(ocp);

  // // reference_trajectory.print();
  // // call the solver
  // returnValue value = solver_.solve();
  // // get solution
  // VariablesGrid output_states, output_control;

  // solver_.getDifferentialStates(output_states);
  // solver_.getControls(output_control);
  // ROS_INFO("[Acado]: Output states: ");
  // output_states.print();
  // for (uint i = 0; i < N; i++) {
  //   _robot_states[i].heading   = output_states(i, 0);
  //   _robot_states[i].pitch     = output_states(i, 1);
  //   _robot_states[i].v_heading = output_states(i, 2);
  //   _robot_states[i].v_pitch   = output_states(i, 3);
  //   _robot_states[i].a_heading = output_control(i, 0);
  //   _robot_states[i].a_pitch   = output_control(i, 1);
  // }

  // heading.clearStaticCounters();
  // v_heading.clearStaticCounters();
  // int success_value = value;
  // ROS_INFO("[Acado]: Acado angular optimization took %.3f, success = %d ", (ros::Time::now() - start).toSec(), success_value);
  // return success_value;
}

std::vector<state> MissionPlanner::initialTrajectory(
    const state &_state){
      std::vector<state> trajectory_to_optimize;
      state aux_point;
      Eigen::Vector3d vel = Eigen::Vector3d::Zero();
      try{
        vel = (goals_.at(0).pos-states_[param_.drone_id].pos)/(goals_.at(0).pos-states_[param_.drone_id].pos).norm();
      }catch(std::out_of_range& e)
      {
        std::cerr << e.what() << std::endl;
      }
      
      for(int i = 0; i<param_.horizon_length; i++){
        aux_point.pos(0) = _state.pos(0)+i*vel(0)*param_.vel_max*param_.step_size;
        aux_point.pos(1) = _state.pos(1)+i*vel(1)*param_.vel_max*param_.step_size;
        aux_point.pos(2) = _state.pos(2)+i*vel(2)*param_.vel_max*param_.step_size;
        trajectory_to_optimize.push_back(aux_point);
      }
    return trajectory_to_optimize;
}


