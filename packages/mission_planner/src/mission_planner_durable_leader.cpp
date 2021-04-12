#include "mission_planner_durable_leader.hpp"

MissionPlannerDurableLeader::MissionPlannerDurableLeader(parameters params)
    : MissionPlanner(params){};
MissionPlannerDurableLeader::~MissionPlannerDurableLeader(){};

std::vector<state> MissionPlannerDurableLeader::initialTrajectory(
    const state &_state){
    std::vector<state> trajectory_to_optimize;
    return trajectory_to_optimize;
}

void MissionPlannerDurableLeader::appendGoal(const state &_new_goal) {
  state goal;
  goal.pos = pointOnCircle(_new_goal.pos,point_to_inspect_, distance_to_inspect_point_);
  goals_.push_back(goal);
}

void MissionPlannerDurableLeader::optimalTrajectory(
    const std::vector<state> &initial_trajectory) {
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

  ocp.subjectTo(ACADO::AT_START, px_ == initial_trajectory[0].pos(0));
  ocp.subjectTo(ACADO::AT_START, py_ == initial_trajectory[0].pos(1));
  ocp.subjectTo(ACADO::AT_START, pz_ == initial_trajectory[0].pos(2));
  ocp.subjectTo(ACADO::AT_START, vx_ == states_[param_.drone_id].vel(0));
  ocp.subjectTo(ACADO::AT_START, vy_ == states_[param_.drone_id].vel(1));
  ocp.subjectTo(ACADO::AT_START, vz_ == states_[param_.drone_id].vel(2));
  // ocp.subjectTo(ACADO::AT_START, ax_ == initial_trajectory[0].acc(0));
  // ocp.subjectTo(ACADO::AT_START, ay_ == initial_trajectory[0].acc(1));
  // ocp.subjectTo(ACADO::AT_START, az_ == initial_trajectory[0].acc(2));

  // setup reference trajectory
  ACADO::VariablesGrid reference_trajectory(6, my_grid_);
  ACADO::DVector reference_point(6);
  Eigen::Vector3d aux_point, point_on_circle;
  for (int k = 0; k < param_.horizon_length; k++) {
    aux_point(0)       = initial_trajectory[k].pos(0);
    aux_point(1)       = initial_trajectory[k].pos(1);
    aux_point(2)       = initial_trajectory[k].pos(2);
    point_on_circle    = pointOnCircle(aux_point, point_to_inspect_, distance_to_inspect_point_);
    reference_point(0) = point_on_circle(0);
    reference_point(1) = point_on_circle(1);
    reference_point(2) = point_on_circle(2);
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
