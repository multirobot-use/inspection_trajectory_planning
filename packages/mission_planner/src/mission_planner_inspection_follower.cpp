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

    // Add the initial point
    aux = initial_pose;
    aux.time_stamp = current_time_;
    trajectory_to_optimize.push_back(aux);

    // Know the elapsed time
    float elapsed_time, formation_angle;
    int current_i;

    for (int i = 0; i < param_.horizon_length; i++){
      // std::cout << "  i == " << i << "  Current time: " << current_time_ << "  Solved traj time: " << solved_trajectories_[inspection_params_.leader_id][i].time_stamp << std::endl;
      if (solved_trajectories_[inspection_params_.leader_id][i].time_stamp > current_time_){
        current_i = i;

        elapsed_time = current_time_ - solved_trajectories_[inspection_params_.leader_id][0].time_stamp;
        std::cout << "  i == " << i << "  Elapsed time: " << elapsed_time << std::endl << std::endl;

        break;
      }
    }

    // Need to know if the trajectory is being described clockwise or anticlockwise
    bool clockwise = MissionPlannerInspectionFollower::isClockwise();

    // Corrector angle: additional angle rotation because of lack synchronization
    // float corrector_angle = MissionPlannerInspectionFollower::calculateAngleCorrector(elapsed_time);
    // float corrector_angle = 0;

    // if (param_.drone_id == 2) {
    //   if (clockwise){  rotation = trajectory_planner::eulerToQuat(0, 0, (relative_angle_ - corrector_angle));}
    //   else{            rotation = trajectory_planner::eulerToQuat(0, 0, -(relative_angle_ + corrector_angle));}
    // }
    // else {
    //   if (clockwise){  rotation = trajectory_planner::eulerToQuat(0, 0, -(relative_angle_ + corrector_angle));}
    //   else{            rotation = trajectory_planner::eulerToQuat(0, 0, (relative_angle_ - corrector_angle));}
    // } 

    if (param_.drone_id == 2) {
      if (clockwise){  rotation = trajectory_planner::eulerToQuat(0, 0, relative_angle_);}
      else{            rotation = trajectory_planner::eulerToQuat(0, 0, -relative_angle_);}
    }
    else {
      if (clockwise){  rotation = trajectory_planner::eulerToQuat(0, 0, -relative_angle_);}
      else{            rotation = trajectory_planner::eulerToQuat(0, 0, relative_angle_);}
    } 
      
    Eigen::Matrix3d rotMat = rotation.toRotationMatrix();
    Eigen::Vector3d aux_point_to_inspect = point_to_inspect_;
    aux_point_to_inspect(2) = 0;
    
    // Add the other points
    for (int i = current_i; i < param_.horizon_length; i++) {
      aux.time_stamp = current_time_ + (i - current_i + 1)*param_.step_size;
      aux.pos = rotMat * (solved_trajectories_[inspection_params_.leader_id][i].pos -
                          aux_point_to_inspect) +
                aux_point_to_inspect;
      trajectory_to_optimize.push_back(std::move(aux));
    }

    formation_angle  = MissionPlannerInspection::getFormationAngle(param_.drone_id);
    
    std::cout << "  Current relative angle (rad): "  << formation_angle
              << "  Desired formation angle (rad): " << relative_angle_
              << "  DIFFERENCE (º): "  << (abs(formation_angle - relative_angle_)*180/(M_PI)) << std::endl << std::endl;

    // std::cout << "  Time of the first point of FOLLOWER: " << trajectory_to_optimize[0].time_stamp << std::endl;
    // std::cout << "  Time of the second point of FOLLOWER: " << trajectory_to_optimize[1].time_stamp << std::endl;
    return trajectory_to_optimize;
  }
  else{
    return trajectory_to_optimize;
  }
}

float MissionPlannerInspectionFollower::calculateAngleCorrector(const float &_elapsed_time){
  // Angle = (distance_traveled_in_elapsed_time / total_curve_length) * 2*pi
  float angle = (_elapsed_time*param_.vel_max)/(distance_to_inspect_point_);

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
