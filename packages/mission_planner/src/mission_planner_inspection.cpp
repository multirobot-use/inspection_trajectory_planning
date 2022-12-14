#include "mission_planner_inspection.hpp"

MissionPlannerInspection::MissionPlannerInspection(const trajectory_planner::parameters _params, const inspection_params _inspection_params)
    : trajectory_planner::TrajectoryPlanner(_params),
      inspection_params_(_inspection_params)
 {
  // initialize reference and solved trajectory
  solved_trajectories_[param_.drone_id] =
      std::vector<state>(param_.horizon_length);
  reference_trajectories_[param_.drone_id] =
      std::vector<state>(param_.horizon_length);

  // initialize logger
  logger_ = std::make_unique<trajectory_planner::Logger>(param_.drone_id);
}

MissionPlannerInspection::~MissionPlannerInspection() {}

Eigen::Vector3d MissionPlannerInspection::pointOnCircleAngle(const Eigen::Vector2d _vector) {
  Eigen::Vector2d point_2D, point_on_circle_2D, inspection_point_2D;
  Eigen::Vector3d point_on_circle_3D;

  point_2D(0) = cos(_vector(0));
  point_2D(1) = sin(_vector(0));

  inspection_point_2D(0) = point_to_inspect_(0);
  inspection_point_2D(1) = point_to_inspect_(1);

  point_on_circle_2D = distance_to_inspect_point_*point_2D +
                       inspection_point_2D;

  point_on_circle_3D(0) = point_on_circle_2D(0);
  point_on_circle_3D(1) = point_on_circle_2D(1);
  point_on_circle_3D(2) = _vector(1);

  return point_on_circle_3D;
}

Eigen::Vector3d MissionPlannerInspection::pointOnCircle(const Eigen::Vector3d _point) {
  Eigen::Vector2d point_2D, inspection_point_2D, point_on_circle_2D;
  Eigen::Vector3d point_on_circle_3D;

  point_2D(0) = _point(0);
  point_2D(1) = _point(1);

  inspection_point_2D(0) = point_to_inspect_(0);
  inspection_point_2D(1) = point_to_inspect_(1);

  point_on_circle_2D = distance_to_inspect_point_ *
                           (point_2D - inspection_point_2D) /
                           ((point_2D - inspection_point_2D).norm()) +
                       inspection_point_2D;

  point_on_circle_3D(0) = point_on_circle_2D(0);
  point_on_circle_3D(1) = point_on_circle_2D(1);
  point_on_circle_3D(2) = _point(2);

  return point_on_circle_3D;
}

float MissionPlannerInspection::calculateFormationAngle(const int &_id){
  float leader_angle   = getAngle(states_[inspection_params_.leader_id].pos, point_to_inspect_);
  float follower_angle = getAngle(states_[_id].pos, point_to_inspect_);

  return getTotalAngle(leader_angle, follower_angle);
}

bool MissionPlannerInspection::isClockwise(const Eigen::Vector3d &_point1, const Eigen::Vector3d &_point2){

  float angle1 = getPointAngle(_point1);
  if (angle1 > 2*M_PI){
    while (angle1 > 2*M_PI){
      angle1 = angle1 - 2*M_PI;
    }
  }

  float angle2 = getPointAngle(_point2);
  if (angle2 > 2*M_PI){
    while (angle2 > 2*M_PI){
      angle2 = angle2 - 2*M_PI;
    }
  }

  float angle_diff = angle2 - angle1;
  
  // std::cout << "Angle diff: " << angle_diff << std::endl;

  if (angle_diff <= -M_PI){       return false;}
  else if (angle_diff >= M_PI){   return true;}
  else if (angle_diff < 0){       return true;}
  else{                           return false;}

}

float MissionPlannerInspection::calculateInspectionDistance(const int &_id){
  Eigen::Vector2d leader_pose, inspection_point;
  leader_pose(0) = states_[_id].pos(0);
  leader_pose(1) = states_[_id].pos(1);
  inspection_point(0) = point_to_inspect_(0);
  inspection_point(1) = point_to_inspect_(1);

  float distance = abs((leader_pose - inspection_point).norm());

  return distance;
}

float MissionPlannerInspection::getTotalAngle(const float &_initial_angle,
                                              const float &_final_angle) {
  float total_angle, initial_angle, final_angle;

  if (_initial_angle < 0)
    initial_angle = _initial_angle + 2 * M_PI;
  else if (_initial_angle > 2*M_PI) {
      initial_angle = _initial_angle;
      while (initial_angle > 2*M_PI){
        initial_angle = initial_angle - 2*M_PI;
      }
  }
  else initial_angle = _initial_angle;

  if (_final_angle < 0)
    final_angle = _final_angle + 2 * M_PI;
  else if (_final_angle > 2*M_PI){
      final_angle = _final_angle;
      while (final_angle > 2*M_PI){
        final_angle = final_angle - 2*M_PI;
      }
  }
  else final_angle = _final_angle;

  // Get the shortest path (should be always between [-M_PI, M_PI])
  if (final_angle - initial_angle > M_PI)
    total_angle = final_angle - (initial_angle + 2 * M_PI);
  if (final_angle - initial_angle < -M_PI)
    total_angle = final_angle + (2 * M_PI - initial_angle);
  else
    total_angle = final_angle - initial_angle;

  return total_angle;
}

void MissionPlannerInspection::appendWaypointAngle(Eigen::Vector2d &_vector) {
  waypoint_angle_height waypoint;

  waypoint.angle  = _vector(0);
  waypoint.height = _vector(1);
  waypoints_angle_.push_back(std::move(waypoint));
}

void MissionPlannerInspection::initialOrientation(std::vector<state> &traj) {
  Eigen::Vector3d aux;
  for (auto &point : traj) {
    aux = point_to_inspect_ - point.pos;
    point.orientation = trajectory_planner::eulerToQuat(0, 0, atan2(aux(1), aux(0)));
  }
}

float MissionPlannerInspection::getPointAngle(const Eigen::Vector3d &_point){
  Eigen::Vector3d relative_position = _point - point_to_inspect_;
  float angle = atan2(relative_position(1), relative_position(0));

  if (angle >= M_PI)   angle = angle + 2*M_PI;

  return angle;
}

float MissionPlannerInspection::calculateVel(const float &_distance){
  // Velocity*Time = Distance
  float vel = (2*M_PI*_distance)/orbit_time_;
  if (vel < param_.vel_min)       vel = param_.vel_min;
  else if (vel > param_.vel_max)  vel = param_.vel_max;

  return vel;
}

bool MissionPlannerInspection::inspecting(){
  float distance = (states_[inspection_params_.leader_id].pos - pointOnCircle(goals_[0].pos)).norm();
  std::cout << "Distance: " << distance << std::endl;

  if (distance > INSPECTING_TOL)  return true;
  else                            return false;

}

float MissionPlannerInspection::minimumFormationAngle(){
  // (3*param_.planning_rate/orbit_time_)*2*pi                   // Get two planning rates as minimum
  if ((3*param_.planning_rate/orbit_time_) > 0.5)  return M_PI;  // If exceeds the half of the circumference
  else                                      return (3*param_.planning_rate/orbit_time_)*2*M_PI;
}

float MissionPlannerInspection::maximumFormationAngle(){
  // Should never exceed M_PI - (x) radians
  // In this case, x = (3*param_.planning_rate/orbit_time_) for safety
  return (M_PI - (3*param_.planning_rate/orbit_time_)*2*M_PI);
}