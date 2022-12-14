#include "mission_planner_inspection_leader.hpp"

MissionPlannerInspectionLeader::MissionPlannerInspectionLeader(trajectory_planner::parameters _params, inspection_params _inspection_params)
    : MissionPlannerInspection(_params, _inspection_params){};
MissionPlannerInspectionLeader::~MissionPlannerInspectionLeader(){};

std::vector<state> MissionPlannerInspectionLeader::initialTrajectory(
    const state &initial_pose) {
  
  // Refresh the goals (waypoints) in case that either the inspection point or the inspection distance have changed
  refreshGoalsExpressedInAngle();

  // Refresh relative_angle_ considering its minimum/maximum value
  float min_formation_angle = MissionPlannerInspection::minimumFormationAngle();
  float max_formation_angle = MissionPlannerInspection::maximumFormationAngle();

  if (min_formation_angle > relative_angle_)      MissionPlannerInspection::setRelativeAngle(min_formation_angle);
  else if (max_formation_angle < relative_angle_) MissionPlannerInspection::setRelativeAngle(max_formation_angle);

  // Trajectory to return in that function
  reference_trajectories_[param_.drone_id].clear();

  // Transform to polar initial and final point
  Eigen::Vector3d initial_pose_polar =
      transformToPolar(initial_pose.pos, point_to_inspect_);
  Eigen::Vector3d final_pose_polar =
      transformToPolar(goals_[0].pos, point_to_inspect_);

  // Calculate curve length and theta_total
  float theta_total  = MissionPlannerInspection::getTotalAngle(initial_pose_polar(1), final_pose_polar(1));
  float curve_length =
      sqrt(pow(distance_to_inspect_point_, 2) * pow(theta_total, 2) +
           pow(final_pose_polar(2) - initial_pose_polar(2), 2));

  Eigen::Vector3d k_point_polar;
  Eigen::Vector3d k_point_xyz;
  float vel, t_k;
  state k_state = initial_pose;

  // Initialize k_point_polar(0) in order to be able to calculate the velocity
  k_point_polar(0) = initial_pose_polar(0);

  // Generate the reference/initial trajectory
  for (int k = 0; k < param_.horizon_length; k++) {
    k_state.time_stamp = start_plan_time_ + param_.planning_rate + k*param_.step_size;

    // Calculate vel and saturate if necessary
    vel = MissionPlannerInspection::calculateVel(k_point_polar(0));

    // Calculate parameter tk
    t_k = (vel * param_.step_size * k) / curve_length;

    // Saturation of t_k value
    if ((t_k > 1) && (stop_))  t_k = 1;

    // Calculate point with parameter tk
    k_point_polar(0) = initial_pose_polar(0) +
                       (final_pose_polar(0) - initial_pose_polar(0)) * t_k;
    k_point_polar(1) = initial_pose_polar(1) + (theta_total)*t_k;
    k_point_polar(2) = initial_pose_polar(2) +
                       (final_pose_polar(2) - initial_pose_polar(2)) * t_k;

    // Polar to cartesians
    k_point_xyz(0) =
        k_point_polar(0) * cos(k_point_polar(1)) + point_to_inspect_(0);
    k_point_xyz(1) =
        k_point_polar(0) * sin(k_point_polar(1)) + point_to_inspect_(1);
    k_point_xyz(2) = k_point_polar(2);
    k_state.pos    = k_point_xyz;  

    // Push back the state
    reference_trajectories_[param_.drone_id].push_back(std::move(k_state));
  }

  return reference_trajectories_[param_.drone_id];
}

std::vector<state> MissionPlannerInspectionLeader::inspectionTrajectory(
    const state &initial_pose){

  refreshGoalsExpressedInAngle();

  // Trajectory to return in that function
  std::vector<state> traj;
  traj.clear();

  // Transform to polar initial and final point
  Eigen::Vector3d initial_pose_polar =
      transformToPolar(initial_pose.pos, point_to_inspect_);
  Eigen::Vector3d final_pose_xyz     = pointOnCircle(goals_[0].pos);
  Eigen::Vector3d final_pose_polar   = transformToPolar(final_pose_xyz, point_to_inspect_);

  // Calculate curve length and theta_total
  float theta_total = MissionPlannerInspection::getTotalAngle(initial_pose_polar(1), final_pose_polar(1));

  // Curve length 
  float curve_length =
      sqrt(pow((final_pose_polar(0) - initial_pose_polar(0)), 2) +
           pow((final_pose_polar(2) - initial_pose_polar(2)), 2));
  float t_k;

  // std::cout << "Curve length: " << curve_length << std::endl;
  if (curve_length < INSPECTING_TOL)   return traj;

  Eigen::Vector3d k_point_polar;
  Eigen::Vector3d k_point_xyz;
  state k_state = initial_pose;

  // Generate the reference/initial trajectory
  for (int k = 0; k < param_.horizon_length; k++) {
    // Calculate parameter tk
    k_state.time_stamp = start_plan_time_ + param_.planning_rate + k*param_.step_size;

    if (curve_length < INSPECTING_TOL)      t_k = 1;
    else                                    t_k = (param_.vel_inspect * param_.step_size * k) / curve_length;

    // Saturation of t_k value
    if (t_k < 0)       t_k = 0;
    else if (t_k > 1)  t_k = 1;

    // Calculate point with parameter tk
    k_point_polar(0) = initial_pose_polar(0) +
                       (final_pose_polar(0) - initial_pose_polar(0)) * t_k;
    k_point_polar(1) = initial_pose_polar(1) + (theta_total)*t_k;
    k_point_polar(2) = initial_pose_polar(2) +
                       (final_pose_polar(2) - initial_pose_polar(2)) * t_k;

    // Polar to cartesians
    k_point_xyz(0) =
        k_point_polar(0) * cos(k_point_polar(1)) + point_to_inspect_(0);
    k_point_xyz(1) =
        k_point_polar(0) * sin(k_point_polar(1)) + point_to_inspect_(1);
    k_point_xyz(2) = k_point_polar(2);
    k_state.pos    = k_point_xyz;

    // Push back the state
    traj.push_back(std::move(k_state));
  }

  return traj;
}

void MissionPlannerInspectionLeader::appendGoal(const state &_new_goal) {
  state goal;
  Eigen::Vector3d goal_vector = _new_goal.pos;

  goal.pos = pointOnCircle(goal_vector);

  goals_.push_back(goal);
}

bool MissionPlannerInspectionLeader::checks() {
  std::cout << "checking... " << std::endl;
  if (!hasGoal()) {
    std::cout << "Mission Planner " << param_.drone_id << " does not have goals"
              << std::endl;
    return false;
  }
  if (!hasPose()) {
    std::cout << "Mission Planner " << param_.drone_id
              << " does not have all poses" << std::endl;
    return false;
  }

  // Check waypoints to remove or not the waypoints to follow
  if (waypointReached(goals_[0], states_[param_.drone_id])) {
    std::cout << "Waypoint reached!" << std::endl;
    last_goal_ = goals_[0];

    // Infer here if the formation has to stop or not
    // Know if the formation has to slow down when it is arriving the waypoint
    
    // In the case of Operation Mode 1, it never stops
    if (operation_mode_ == trajectory_planner::OperationMode::NONSTOP) {
      stop_ = false;
      }

    // In the case of Operation Mode 2, if there are at least 2 waypoints (plus the current), it is important to verify
    // if the direction that has to follow is the same in both cases
    if (operation_mode_ == trajectory_planner::OperationMode::SMOOTH){
      if (goals_.size() >= 3){
        bool direction1 = MissionPlannerInspection::isClockwise(goals_[0].pos, goals_[1].pos);
        bool direction2 = MissionPlannerInspection::isClockwise(goals_[1].pos, goals_[2].pos);

        // std::cout << "Direction 1: " << direction1 << "    Direction 2: " << direction2 << std::endl;
        if (direction1 == direction2)   stop_ = false;
        else                            stop_ = true;
      }
      else{
        stop_ = false;
      }
    }

    // In the case of Operation Mode 3 and 4, it always stops
    if (operation_mode_ == trajectory_planner::OperationMode::STOP){
      stop_ = true;
    }

    if (operation_mode_ == trajectory_planner::OperationMode::INSPECT){
      planner_state_ = trajectory_planner::PlannerStatus::INSPECTING;
      std::cout << "Inspecting..." << std::endl;
      stop_ = true;
    }

    if ((operation_mode_ != trajectory_planner::OperationMode::INSPECT) &&
        (planner_state_ == trajectory_planner::PlannerStatus::INSPECTING)) {
      std::cout << "Not inspecting anymore" << std::endl;
      planner_state_ = trajectory_planner::PlannerStatus::REPLANNED;
    }

    if (planner_state_ != trajectory_planner::PlannerStatus::INSPECTING ||
       (planner_state_ == trajectory_planner::PlannerStatus::INSPECTING && skip_ == true)){
      
      skip_ = false;
      planner_state_ = trajectory_planner::PlannerStatus::REPLANNED;
      std::cout << "Removed waypoint" << std::endl;
      init_point_ = goals_[0].pos;
      goals_.erase(goals_.begin());
      waypoints_angle_.erase(waypoints_angle_.begin());

      if (!hasGoal()) return false;
    }
  }
  std::cout << "Stop: " << stop_ << std::endl;

  return true;
}