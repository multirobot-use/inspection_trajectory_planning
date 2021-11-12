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
    trajectory_to_optimize.push_back(initial_pose);

    // Corrector angle: additional angle rotation because of lack synchronization
    float corrector_angle = MissionPlannerInspectionFollower::calculateAngleCorrector();
    // float corrector_angle = 0;

    // Need to know if the trajectory is being described clockwise or anticlockwise
    bool clockwise = MissionPlannerInspectionFollower::isClockwise();

    if (param_.drone_id == 2) {
      if (clockwise){  rotation = trajectory_planner::eulerToQuat(0, 0, (relative_angle_ - corrector_angle));}
      else{            rotation = trajectory_planner::eulerToQuat(0, 0, -(relative_angle_ + corrector_angle));}
    }
    else {
      if (clockwise){  rotation = trajectory_planner::eulerToQuat(0, 0, -(relative_angle_ + corrector_angle));}
      else{            rotation = trajectory_planner::eulerToQuat(0, 0, (relative_angle_ - corrector_angle));}
    }
      
    Eigen::Matrix3d rotMat = rotation.toRotationMatrix();
    Eigen::Vector3d aux_point_to_inspect = point_to_inspect_;
    aux_point_to_inspect(2) = 0;
    
    for (int i = 1; i < param_.horizon_length; i++) {
      aux.pos = rotMat * (solved_trajectories_[inspection_params_.leader_id][i].pos -
                          aux_point_to_inspect) +
                aux_point_to_inspect;
      trajectory_to_optimize.push_back(std::move(aux));
    }

    return trajectory_to_optimize;
  }
  else{
    return trajectory_to_optimize;
  }
}

float MissionPlannerInspectionFollower::calculateAngleCorrector(){
  // Angle = (distance_traveled_in_planning_rate / total_curve_length) * 2*pi
  float angle = (param_.planning_rate*param_.vel_max)/(distance_to_inspect_point_);

  return angle;
}

bool MissionPlannerInspectionFollower::isClockwise(){
  // Know the direction based on the angle of first point and of the second point of the leader's trajectory
  Eigen::Vector3d point1 = solved_trajectories_[inspection_params_.leader_id][1].pos;
  Eigen::Vector3d point2 = solved_trajectories_[inspection_params_.leader_id][2].pos;

  float angle1 = getPointAngle(point1);
  float angle2 = getPointAngle(point2);
  float angle_diff = angle2 - angle1;

  if (angle_diff < -M_PI){         std::cout << "ANTICLOCKWISE" << std::endl;  return false;}
  else if (angle_diff > M_PI){     std::cout << "CLOCKWISE" << std::endl;      return true;}
  else if (angle_diff < 0){        std::cout << "CLOCKWISE" << std::endl;      return true;}
  else{                            std::cout << "ANTICLOCKWISE" << std::endl;  return false;}

}
