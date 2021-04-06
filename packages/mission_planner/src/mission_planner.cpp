#include "mission_planner.hpp"

MissionPlanner::MissionPlanner(const parameters _param)
    : param_(_param),
      last_trajectory_(_param.horizon_length),
      my_grid_(0.0, (param_.horizon_length - 1) * param_.step_size,
               param_.horizon_length) {}

MissionPlanner::~MissionPlanner() {}

void MissionPlanner::appendGoal(const state &_new_goal) {
  goals_.push_back(_new_goal);

  // // Test
  // int goals_size = (goals_.size()-1);

  // for (int i=0; i < goals_size; i++)
  // {
  //   ROS_INFO("Already have point %d position x:   %f", i, goals_[i].pos[0]);
  // }
  // ROS_INFO("Receiving now point %d position x:   %f", goals_size, goals_[goals_size].pos[0]);
}

void MissionPlanner::setPointToInspect(const Eigen::Vector3d &point){
  point_to_inspect_ = point;
  std::cout << "New point to inspect added!";
}

void MissionPlanner::clearGoals() { goals_.clear(); }

void MissionPlanner::plan() {
  if(goals_.empty()){
    std::cout<<"Mission planner "<<param_.drone_id<<": no goals"<<std::endl;
    return;
  }

  state initial_pose;
  if(states_.count(param_.drone_id) == 0){
      std::cout<<"uav "<<param_.drone_id<<"pose is not available"<<std::endl;
      return;
  } 

  if (planner_state_ == PlannerStatus::FIRST_PLAN) {
    initial_pose = states_[param_.drone_id];
  } else if (planner_state_ == PlannerStatus::REPLANNED) {
    initial_pose = last_trajectory_[param_.planning_rate];
  }
  // check waypoints to remove or not the waypoints to follow
  // checkWaypoints();
  // calculate initial trajectory
  std::vector<state> initial_traj = initialTrajectory(initial_pose);
  // calculate optimal trajectory
  optimalTrajectory(initial_traj);
}

void MissionPlanner::checkWaypoints(){};
