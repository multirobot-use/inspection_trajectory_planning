#include "mission_planner_inspection_leader.hpp"

MissionPlannerInspectionLeader::MissionPlannerInspectionLeader(trajectory_planner::parameters _params, inspection_params _inspection_params)
    : MissionPlannerInspection(_params, _inspection_params){};
MissionPlannerInspectionLeader::~MissionPlannerInspectionLeader(){};

std::vector<state> MissionPlannerInspectionLeader::initialTrajectory(
    const state &initial_pose) {
  
  // Refresh the goals (waypoints) in case that either the inspection point or the inspection distance have changed
  refreshGoals();

  std::vector<state> traj;  // trajectory to return in that function

  // transform to polar initial and final point
  Eigen::Vector3d initial_pose_polar =
      transformToPolar(initial_pose.pos, point_to_inspect_);
  Eigen::Vector3d final_pose_polar =
      transformToPolar(goals_[0].pos, point_to_inspect_);

  // calculate curve length and theta_total
  float theta_total = getTotalAngle(initial_pose_polar(1), final_pose_polar(1));
  float curve_length =
      sqrt(pow(distance_to_inspect_point_, 2) * pow(theta_total, 2) +
           pow(final_pose_polar(2) - initial_pose_polar(2), 2));

  Eigen::Vector3d k_point_polar;
  Eigen::Vector3d k_point_xyz;
  state k_state = initial_pose;
  
  // Generate the reference/initial trajectory
  for (int k = 0; k < param_.horizon_length; k++) {
    // calculate parameter tk
    k_state.time_stamp = current_time_ + k*param_.step_size;
    float t_k = (param_.vel_max * param_.step_size * k) / curve_length;

    // Saturation of t_k value
    // if (t_k > 1)  t_k = 1;

    // calculate point with parameter tk
    k_point_polar(0) = initial_pose_polar(0) +
                       (final_pose_polar(0) - initial_pose_polar(0)) * t_k;
    k_point_polar(1) = initial_pose_polar(1) + (theta_total)*t_k;
    k_point_polar(2) = initial_pose_polar(2) +
                       (final_pose_polar(2) - initial_pose_polar(2)) * t_k;

    // polar to cartesians
    k_point_xyz(0) =
        k_point_polar(0) * cos(k_point_polar(1)) + point_to_inspect_(0);
    k_point_xyz(1) =
        k_point_polar(0) * sin(k_point_polar(1)) + point_to_inspect_(1);
    k_point_xyz(2) = k_point_polar(2);
    k_state.pos = k_point_xyz;  

    // push back the state
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

float MissionPlannerInspectionLeader::getTotalAngle(const float &_initial_angle,
                                                 const float &_final_angle) {
  float total_angle, initial_angle, final_angle;

  if (_initial_angle < 0)
    initial_angle = _initial_angle + 2 * M_PI;
  else
    initial_angle = _initial_angle;
  if (_final_angle < 0)
    final_angle = _final_angle + 2 * M_PI;
  else
    final_angle = _final_angle;

  // Get the shortest path (should be always between [-M_PI, M_PI])
  if (final_angle - initial_angle > M_PI)
    total_angle = final_angle - (initial_angle + 2 * M_PI);
  if ((final_angle - initial_angle < M_PI) && (final_angle - initial_angle < 0))
    total_angle = final_angle - initial_angle;
  if (final_angle - initial_angle < -M_PI)
    total_angle = final_angle + (2 * M_PI - initial_angle);
  if ((final_angle - initial_angle < M_PI) && (final_angle - initial_angle > 0))
    total_angle = final_angle - initial_angle;

  return total_angle;
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

  // check waypoints to remove or not the waypoints to follow
  if (waypointReached(goals_[0], states_[param_.drone_id])) {
    std::cout << "Removed waypoint" << std::endl;
    init_point_ = goals_[0].pos;
    goals_.erase(goals_.begin());
    if (!hasGoal()) return false;
  }
  return true;
}