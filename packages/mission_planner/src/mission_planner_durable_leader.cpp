#include "mission_planner_durable_leader.hpp"

MissionPlannerDurableLeader::MissionPlannerDurableLeader(parameters params)
    : MissionPlanner(params){};
MissionPlannerDurableLeader::~MissionPlannerDurableLeader(){};

<<<<<<< HEAD
std::vector<state> MissionPlannerDurableLeader::initialTrajectory(
    const state &_state){
    std::vector<state> trajectory_to_optimize;


  // ACADO::DifferentialState px_, py_, pz_, vx_, vy_, vz_;
  // ACADO::Control ax_, ay_, az_;
  
  // ACADO::DifferentialEquation model;

  // // define the model
  // model << dot(px_) == vx_;
  // model << dot(py_) == vy_;
  // model << dot(pz_) == vz_;
  // model << dot(vx_) == ax_;
  // model << dot(vy_) == ay_;
  // model << dot(vz_) == az_;

  // ACADO::OCP ocp(my_grid_);
  // ocp.subjectTo(model);

  // ocp.subjectTo(  -param_.acc_max <= ax_ <= param_.acc_max   );  
  // ocp.subjectTo(  -param_.acc_max <= ay_ <= param_.acc_max   );
  // ocp.subjectTo(  -param_.acc_max <= az_ <= param_.acc_max   );
  // ocp.subjectTo(  -param_.vel_max <= vx_ <= param_.vel_max   );
  // ocp.subjectTo(  -param_.vel_max <= vy_ <= param_.vel_max   );
  // ocp.subjectTo(  -param_.vel_max <= vz_ <= param_.vel_max   );

  // ocp.subjectTo(ACADO::AT_START, px_ == _state.pos(0));
  // ocp.subjectTo(ACADO::AT_START, py_ == _state.pos(1));
  // ocp.subjectTo(ACADO::AT_START, pz_ == _state.pos(2));
  // ocp.subjectTo(ACADO::AT_START, vx_ == _state.vel(0));
  // ocp.subjectTo(ACADO::AT_START, vy_ == _state.vel(1));
  // ocp.subjectTo(ACADO::AT_START, vz_ == _state.vel(2));
  // // ocp.subjectTo(ACADO::AT_START, ax_ == _state.acc(0));
  // // ocp.subjectTo(ACADO::AT_START, ay_ == _state.acc(1));
  // // ocp.subjectTo(ACADO::AT_START, az_ == _state.acc(2));

  // // setup reference trajectory
  // ACADO::VariablesGrid reference_trajectory(6, my_grid_);
  // ACADO::DVector reference_point(6);
  // Eigen::Vector3d aux_point, point_on_circle;
  // for (int k = 0; k < goals_.size(); k++) {
  //   aux_point(0) = goals_[k].pos(0);
  //   aux_point(1) = goals_[k].pos(1);
  //   aux_point(2) = goals_[k].pos(2);
  //   point_on_circle = pointOnCircle(aux_point, point_to_inspect_, distance_to_inspect_point_);
  //   reference_point(0) = point_on_circle(0);
  //   reference_point(1) = point_on_circle(1);
  //   reference_point(2) = point_on_circle(2);
  //   reference_point(3) = 0.0;
  //   reference_point(4) = 0.0;
  //   reference_point(5) = 0.0;
  //   reference_trajectory.setVector(k, reference_point);  // TODO: check
  // }

  // // DEFINE LSQ function to minimize diff from desired trajectory
  // ACADO::Function rf;

  // rf << px_ << py_ << pz_ << ax_ << ay_ << az_;

  // ACADO::DMatrix S(6, 6);

  // S.setIdentity();

  // ocp.minimizeLSQ(S, rf, reference_trajectory);

  // ACADO::OptimizationAlgorithm solver(ocp);

  // solver.set(ACADO::MAX_TIME, 2.0); // TODO: have it as parameter
  
  // bool solver_success = solver.solve();
  
  // ACADO::VariablesGrid output_states, output_control;

  // solver.getDifferentialStates(output_states);
  // solver.getControls(output_control);
  // for (int k = 0; k < (param_.horizon_length - 1) * param_.step_size; k++) {
  //   trajectory_to_optimize[k].pos(0) = output_states(k, 0);
  //   trajectory_to_optimize[k].pos(1) = output_states(k, 1);
  //   trajectory_to_optimize[k].pos(2) = output_states(k, 2);
  //   trajectory_to_optimize[k].vel(0) = output_states(k, 3);
  //   trajectory_to_optimize[k].vel(1) = output_states(k, 4);
  //   trajectory_to_optimize[k].vel(2) = output_states(k, 5);
  //   trajectory_to_optimize[k].acc(0) = output_control(k, 0);
  //   trajectory_to_optimize[k].acc(1) = output_control(k, 1);
  //   trajectory_to_optimize[k].acc(2) = output_control(k, 2);
  // }
  

  // px_.clearStaticCounters();
  // py_.clearStaticCounters();
  // pz_.clearStaticCounters();
  // vx_.clearStaticCounters();
  // vy_.clearStaticCounters();
  // vz_.clearStaticCounters();
  // ax_.clearStaticCounters();
  // ay_.clearStaticCounters();
  // az_.clearStaticCounters();


    return trajectory_to_optimize;
}
=======
>>>>>>> b6fdbd9a5ce1d17fd8860cd85a756413260d7367

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
