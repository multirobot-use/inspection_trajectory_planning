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

  if (!solved_trajectories_[inspection_params_.leader_id].empty()) {
    refreshGoals();
    state aux;

    // Need to know the elapsed time
    float elapsed_time;
    int current_i = 0;
    bool solved = false;

    for (int i = 0; i < param_.horizon_length; i++){
      if (solved_trajectories_[inspection_params_.leader_id][i].time_stamp > (start_plan_time_ + param_.planning_rate)){
        current_i = i;
        solved = true;
        elapsed_time = start_plan_time_ - solved_trajectories_[inspection_params_.leader_id][0].time_stamp;
        std::cout << "  i == " << i << "  Elapsed time: " << elapsed_time << std::endl << std::endl;

        break;
      }
    }

    if (!solved) return trajectory_to_optimize;

    // Add the initial point
    aux = initial_pose;
    aux.time_stamp = start_plan_time_ + param_.planning_rate;
    trajectory_to_optimize.push_back(aux);

    // Need to know if the trajectory is being described clockwise or anticlockwise
    bool clockwise = MissionPlannerInspection::isClockwise(solved_trajectories_[inspection_params_.leader_id][1].pos,
                                                           solved_trajectories_[inspection_params_.leader_id][2].pos);

    // Corrector angle: additional angle rotation because of lack synchronization
    // float corrector_angle = MissionPlannerInspectionFollower::calculateAngleCorrector(elapsed_time);
    // float corrector_angle = 0;

    // if (param_.drone_id == 2) {
    //   if (clockwise){  rotation = trajectory_planner::eulerToQuat(0, 0, (relative_angle_ - corrector_angle));}
    //   else{            rotation = trajectory_planner::eulerToQuat(0, 0, (relative_angle_ + corrector_angle));}
    // }
    // else {
    //   if (clockwise){  rotation = trajectory_planner::eulerToQuat(0, 0, (relative_angle_ + corrector_angle));}
    //   else{            rotation = trajectory_planner::eulerToQuat(0, 0, (relative_angle_ - corrector_angle));}
    // } 

    if (param_.drone_id == 2) {
      rotation = trajectory_planner::eulerToQuat(0, 0, relative_angle_);
    }
    else {
      rotation = trajectory_planner::eulerToQuat(0, 0, -relative_angle_);
    } 
      
    Eigen::Matrix3d rotMat = rotation.toRotationMatrix();
    Eigen::Vector3d aux_point_to_inspect = point_to_inspect_;
    aux_point_to_inspect(2) = 0;
    
    // Add the other points
    for (int i = current_i; i < param_.horizon_length; i++) {
      aux.time_stamp = start_plan_time_ + param_.planning_rate + (i - current_i + 1)*param_.step_size;
      aux.pos = rotMat * (solved_trajectories_[inspection_params_.leader_id][i].pos -
                          aux_point_to_inspect) +
                aux_point_to_inspect;
      trajectory_to_optimize.push_back(std::move(aux));
    }

    formation_angle_[param_.drone_id] = MissionPlannerInspection::calculateFormationAngle(param_.drone_id);
    
    std::cout << "  Current relative angle (rad): "  << formation_angle_[param_.drone_id]
              << "  Desired formation angle (rad): " << relative_angle_
              << "  DIFFERENCE (º): "  << (abs(formation_angle_[param_.drone_id] - relative_angle_)*180/(M_PI)) << std::endl << std::endl;

    return trajectory_to_optimize;
  }
  else{
    return trajectory_to_optimize;
  }
}

std::vector<state> MissionPlannerInspectionFollower::inspectionTrajectory(
    const state &initial_pose){

  // Trajectory to return in that function
  std::vector<state> traj;
  traj.clear();

  // Transform to polar initial and final point
  Eigen::Vector3d initial_pose_polar =
      transformToPolar(initial_pose.pos, point_to_inspect_);
  
  // DIFFERENT final_pose_polar
  Eigen::Vector3d final_pose_xyz   = pointOnCircle(last_goal_.pos);
  Eigen::Vector3d final_pose_polar = transformToPolar(final_pose_xyz, point_to_inspect_);
  
  // Calculate curve length and theta_total
  float theta_total = MissionPlannerInspection::getTotalAngle(initial_pose_polar(1), final_pose_polar(1));
  float curve_length =
      sqrt(pow(distance_to_inspect_point_, 2) * pow(theta_total, 2) +
           pow(final_pose_polar(2) - initial_pose_polar(2), 2));
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

    // Change 0.15 por param_.vel_inspecting
    if (curve_length < INSPECTING_TOL)      t_k = 0;
    else                                    t_k = (0.15 * param_.step_size * k) / curve_length;

    // Saturation of t_k value (Uncomment in order to slow down while is arriving the waypoint. Not overshooting behaviour)
    if (t_k < 0)  t_k = 0;
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

    // if (k > 35)  std::cout << "k: " << k << "  [X, Y, Z]: [" << k_point_xyz(0) << ", " << k_point_xyz(1) << ", " << k_point_xyz(2) << "]" << std::endl;

    // Push back the state
    traj.push_back(std::move(k_state));
  }

  return traj;
}

float MissionPlannerInspectionFollower::calculateAngleCorrector(const float &_elapsed_time){
  // Angle = (distance_traveled_in_elapsed_time / total_curve_length) * 2*pi
  float angle = (_elapsed_time*param_.vel_max)/(distance_to_inspect_point_);

  return angle;
}
