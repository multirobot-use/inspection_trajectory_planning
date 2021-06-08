#include "mission_planner_inspection_leader.hpp"

MissionPlannerInspectionLeader::MissionPlannerInspectionLeader(parameters params)
    : MissionPlanner(params){};
MissionPlannerInspectionLeader::~MissionPlannerInspectionLeader(){};

std::vector<state> MissionPlannerInspectionLeader::initialTrajectoryToInspect(
    const state &initial_pose) {
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

  for (int k = 0; k < param_.horizon_length; k++) {
    // calculate parameter tk
    float t_k = (param_.vel_max * param_.step_size * k) / curve_length;

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

bool MissionPlannerInspectionLeader::isClockWise(const Eigen::Vector3d &_vector,
                                              const state &_state) {
  float angle = getAngle(_state.pos, point_to_inspect_);
  int quadrant;

  if ((angle >= -M_PI) && (angle < -M_PI / 2)) quadrant = 3;
  if ((angle >= -M_PI / 2) && (angle < 0)) quadrant = 4;
  if ((angle >= 0) && (angle < M_PI / 2)) quadrant = 1;
  if ((angle >= M_PI / 2) && (angle <= M_PI)) quadrant = 2;

  // std::cout << std::endl << "QUADRANT: " << std::to_string(quadrant) <<
  // std::endl; std::cout << std::endl << "ANGLE (rad): " << angle << std::endl;

  // true if clockwise, false if anticlockwise
  switch (quadrant) {
    case 1:
      if ((_vector(0) > 0) && (_vector(1) < 0))
        return true;
      else
        return false;
      break;

    case 2:
      if ((_vector(0) > 0) && (_vector(1) > 0))
        return true;
      else
        return false;
      break;

    case 3:
      if ((_vector(0) < 0) && (_vector(1) > 0))
        return true;
      else
        return false;
      break;

    case 4:
      if ((_vector(0) < 0) && (_vector(1) < 0))
        return true;
      else
        return false;
      break;

    default:
      break;
  }
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