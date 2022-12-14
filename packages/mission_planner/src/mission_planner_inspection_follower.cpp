#include <mission_planner_inspection_follower.hpp>

MissionPlannerInspectionFollower::MissionPlannerInspectionFollower(trajectory_planner::parameters _params, inspection_params _inspection_params)
    : MissionPlannerInspection(_params, _inspection_params) {}

MissionPlannerInspectionFollower::~MissionPlannerInspectionFollower() {}

bool MissionPlannerInspectionFollower::checks() {
  if (!hasPose()) {
    std::cout << "Mission Planner " << param_.drone_id
              << " does not have all poses" << std::endl;
    return false;
  }
  return true;
}

std::vector<state> MissionPlannerInspectionFollower::initialTrajectory(
    const state &initial_pose) {
  
  std::vector<state> trajectory_to_optimize;
  Eigen::Quaterniond rotation;

  if (!reference_trajectories_[inspection_params_.leader_id].empty()) {
    refreshGoalsExpressedInAngle();
    state aux;

    // Need to know the elapsed time
    float elapsed_time;
    int current_i = 0;
    bool solved   = false;

    for (int i = 0; i < param_.horizon_length; i++){
      if (reference_trajectories_[inspection_params_.leader_id][i].time_stamp > (start_plan_time_ + param_.planning_rate)){
        current_i    = i;
        solved       = true;
        elapsed_time = start_plan_time_ - reference_trajectories_[inspection_params_.leader_id][0].time_stamp;

        std::cout << "  i == " << i << "  Elapsed time: " << elapsed_time << std::endl << std::endl;

        break;
      }
    }

    if (!solved) return trajectory_to_optimize;

    // Add the initial point
    aux            = initial_pose;
    aux.time_stamp = start_plan_time_ + param_.planning_rate;
    trajectory_to_optimize.push_back(aux);

    // Need to know if the trajectory is being described clockwise or anticlockwise
    bool clockwise = MissionPlannerInspection::isClockwise(reference_trajectories_[inspection_params_.leader_id][1].pos,
                                                           reference_trajectories_[inspection_params_.leader_id][2].pos);

    // Previous check if the relative_angle_ is safe by using minimums and maximums values
    float min_formation_angle = MissionPlannerInspection::minimumFormationAngle();
    float max_formation_angle = MissionPlannerInspection::maximumFormationAngle();

    if      (min_formation_angle > relative_angle_)  MissionPlannerInspection::setRelativeAngle(min_formation_angle);
    else if (max_formation_angle < relative_angle_)  MissionPlannerInspection::setRelativeAngle(max_formation_angle);

    if (param_.drone_id == 2) {
      rotation = trajectory_planner::eulerToQuat(0, 0, relative_angle_);
    }
    else {
      rotation = trajectory_planner::eulerToQuat(0, 0, -relative_angle_);
    } 
      
    Eigen::Matrix3d rotMat               = rotation.toRotationMatrix();
    Eigen::Vector3d aux_point_to_inspect = point_to_inspect_;

    // Z coordinate to 0 (by now)
    aux_point_to_inspect(2) = 0;
    
    // Add the other points
    for (int i = current_i; i < param_.horizon_length; i++) {
      aux.time_stamp = start_plan_time_ + param_.planning_rate + (i - current_i + 1)*param_.step_size;
      aux.pos        = rotMat * (reference_trajectories_[inspection_params_.leader_id][i].pos -
                                 aux_point_to_inspect) + aux_point_to_inspect;
      trajectory_to_optimize.push_back(std::move(aux));
    }

    formation_angle_[param_.drone_id] = MissionPlannerInspection::calculateFormationAngle(param_.drone_id);
    
    std::cout << "  Current relative angle (rad): "  << abs(formation_angle_[param_.drone_id])
              << "  Desired formation angle (rad): " << relative_angle_
              << "  DIFFERENCE (??): "  << (abs(abs(formation_angle_[param_.drone_id]) - relative_angle_)*180/(M_PI)) << std::endl << std::endl;

    return trajectory_to_optimize;
  }
  else{
    return trajectory_to_optimize;
  }
}

std::vector<state> MissionPlannerInspectionFollower::inspectionTrajectory(
    const state &initial_pose){
  // Refresh the goals before anything else
  refreshGoalsExpressedInAngle();

  // Trajectory to return in that function
  std::vector<state> traj;
  traj.clear();

  // Transform to polar initial and final point
  Eigen::Vector3d initial_pose_polar = transformToPolar(initial_pose.pos, point_to_inspect_);
  
  // DIFFERENT final_pose_polar
  Eigen::Vector3d final_pose_xyz   = pointOnCircle(goals_[0].pos);
  Eigen::Vector3d final_pose_polar = transformToPolar(final_pose_xyz, point_to_inspect_);
  
  // Formation angle
  float formation_angle;

  if (param_.drone_id == 2) { formation_angle = relative_angle_; }
  else                      { formation_angle = -relative_angle_; }

  // Calculate curve length and theta_total
  float theta_total = MissionPlannerInspection::getTotalAngle(initial_pose_polar(1), final_pose_polar(1) + formation_angle);

  // Approx
  float curve_length =
      sqrt(pow((final_pose_polar(0) - initial_pose_polar(0)), 2) +
           pow(distance_to_inspect_point_, 2) * pow(theta_total, 2) +
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
    k_state.pos = k_point_xyz;

    // Push back the state
    traj.push_back(std::move(k_state));
  }

  return traj;
}

