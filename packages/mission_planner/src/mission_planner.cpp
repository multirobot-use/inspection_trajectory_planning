#include "mission_planner.hpp"

MissionPlanner::MissionPlanner(const parameters _param)
    : param_(_param),
      last_trajectory_(_param.horizon_length),
      my_grid_(0.0, (param_.horizon_length - 1) * param_.step_size,
               param_.horizon_length) {}

MissionPlanner::~MissionPlanner() {}

void MissionPlanner::plan() {
  
  if(!checks()) return;

  state initial_pose = states_[param_.drone_id];

  // if (planner_state_ == PlannerStatus::FIRST_PLAN) {
  //   initial_pose = states_[param_.drone_id];
  // } else if (planner_state_ == PlannerStatus::REPLANNED) {
  //   initial_pose = last_trajectory_[param_.planning_rate];
  // }
  std::vector<state> initial_traj;
  // calculate initial trajectory
  if(isInspectionZone(states_[param_.drone_id].pos)){
    // std::cout<<"TODO: implement trajectory to Inspect"<<std::endl;
    // return;
    initial_traj = initialTrajectoryToInspect();
  }else{
    initial_traj = initialTrajectory(initial_pose);
  }

  if(initial_traj.empty()){
    std::cout<<"Initial trajectory empty...breaking"<<std::endl;
  }else{
    if(trajectoryHasNan(initial_traj)){
      std::cout<<"Initial trajectory has nan"<<std::endl;
      return;
    }
  // calculate optimal trajectory
  optimalTrajectory(initial_traj);
  }
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

std::vector<state> MissionPlanner::pathFromPointToAnother(const Eigen::Vector3d &initial, const Eigen::Vector3d &final){
  std::vector<state> trajectory_to_optimize;
  state aux_point;
  const Eigen::Vector3d vel((final-initial)/(final-initial).norm());
  for(int i = 0; i<param_.horizon_length; i++){
    aux_point.pos =initial+  i*vel*param_.vel_max*param_.step_size;
    trajectory_to_optimize.push_back(std::move(aux_point));
  }
  return trajectory_to_optimize;
}

void MissionPlanner::optimalTrajectory(const std::vector<state> &initial_trajectory){
  ACADO::DifferentialState px_, py_, pz_, vx_, vy_, vz_;
  ACADO::Control ax_, ay_, az_;
  
  ACADO::DifferentialEquation model;

  // define the model
  model << dot(px_) == vx_;
  model << dot(py_) == vy_;
  model << dot(pz_) == vz_;
  model << dot(vx_) == ax_;
  model << dot(vy_) == ay_;
  model << dot(vz_) == az_;

  ACADO::OCP ocp(my_grid_);
  ocp.subjectTo(model);

  ocp.subjectTo(  -param_.acc_max <= ax_ <= param_.acc_max   );  
  ocp.subjectTo(  -param_.acc_max <= ay_ <= param_.acc_max   );
  ocp.subjectTo(  -param_.acc_max <= az_ <= param_.acc_max   );
  ocp.subjectTo(  -param_.vel_max <= vx_ <= param_.vel_max   );
  ocp.subjectTo(  -param_.vel_max <= vy_ <= param_.vel_max   );
  ocp.subjectTo(  -param_.vel_max <= vz_ <= param_.vel_max   );

  ocp.subjectTo(ACADO::AT_START, px_ == states_[param_.drone_id].pos(0));
  ocp.subjectTo(ACADO::AT_START, py_ == states_[param_.drone_id].pos(1));
  ocp.subjectTo(ACADO::AT_START, pz_ == states_[param_.drone_id].pos(2));
  ocp.subjectTo(ACADO::AT_START, vx_ == states_[param_.drone_id].vel(0));
  ocp.subjectTo(ACADO::AT_START, vy_ == states_[param_.drone_id].vel(1));
  ocp.subjectTo(ACADO::AT_START, vz_ == states_[param_.drone_id].vel(2));
  // ocp.subjectTo(ACADO::AT_START, ax_ == initial_trajectory[0].acc(0));
  // ocp.subjectTo(ACADO::AT_START, ay_ == initial_trajectory[0].acc(1));
  // ocp.subjectTo(ACADO::AT_START, az_ == initial_trajectory[0].acc(2));

  // setup reference trajectory
  ACADO::VariablesGrid reference_trajectory(6, my_grid_);
  ACADO::DVector reference_point(6);
  for (int k = 0; k < param_.horizon_length; k++) {
    reference_point(0) = initial_trajectory[k].pos(0);
    reference_point(1) = initial_trajectory[k].pos(1);
    reference_point(2) = initial_trajectory[k].pos(2);
    reference_point(3) = 0.0;
    reference_point(4) = 0.0;
    reference_point(5) = 0.0;
    reference_trajectory.setVector(k, reference_point);  // TODO: check
  }

  // DEFINE LSQ function to minimize diff from desired trajectory
  ACADO::Function rf;

  rf << px_ << py_ << pz_ << ax_ << ay_ << az_;

  ACADO::DMatrix S(6, 6);

  S.setIdentity();

  ocp.minimizeLSQ(S, rf, reference_trajectory);

  ACADO::OptimizationAlgorithm solver(ocp);

  solver.set(ACADO::MAX_TIME, 2.0); // TODO: have it as parameter
  
  bool solver_success = solver.solve();
  
  ACADO::VariablesGrid output_states, output_control;

  solver.getDifferentialStates(output_states);
  solver.getControls(output_control);
  for (int k = 0; k < param_.horizon_length; k++) {
    last_trajectory_[k].pos(0) = output_states(k, 0);
    last_trajectory_[k].pos(1) = output_states(k, 1);
    last_trajectory_[k].pos(2) = output_states(k, 2);
    last_trajectory_[k].vel(0) = output_states(k, 3);
    last_trajectory_[k].vel(1) = output_states(k, 4);
    last_trajectory_[k].vel(2) = output_states(k, 5);
    last_trajectory_[k].acc(0) = output_control(k, 0);
    last_trajectory_[k].acc(1) = output_control(k, 1);
    last_trajectory_[k].acc(2) = output_control(k, 2);
  }
  px_.clearStaticCounters();
  py_.clearStaticCounters();
  pz_.clearStaticCounters();
  vx_.clearStaticCounters();
  vy_.clearStaticCounters();
  vz_.clearStaticCounters();
  ax_.clearStaticCounters();
  ay_.clearStaticCounters();
  az_.clearStaticCounters();
};

bool MissionPlanner::isInspectionZone(const Eigen::Vector3d &drone_pose){
  if((drone_pose- point_to_inspect_).norm()>distance_to_inspect_point_+REACH_TOL)return false;
  else return true;
}


