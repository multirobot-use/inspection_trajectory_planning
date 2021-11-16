#include "mission_planner_inspection.hpp"

MissionPlannerInspection::MissionPlannerInspection(const trajectory_planner::parameters _params, const inspection_params _inspection_params)
    : trajectory_planner::TrajectoryPlanner(_params),
      inspection_params_(_inspection_params)
 {
  // initialize solved trajectory
  solved_trajectories_[param_.drone_id] =
      std::vector<state>(param_.horizon_length);

  // initialize logger
  logger_ = std::make_unique<trajectory_planner::Logger>(param_.drone_id);
}

MissionPlannerInspection::~MissionPlannerInspection() {}

Eigen::Vector3d MissionPlannerInspection::pointOnCircle(const Eigen::Vector3d point) {
  Eigen::Vector2d point_2D, inspection_point_2D, point_on_circle_2D;
  Eigen::Vector3d point_on_circle_3D;

  point_2D(0) = point(0);
  point_2D(1) = point(1);

  inspection_point_2D(0) = point_to_inspect_(0);
  inspection_point_2D(1) = point_to_inspect_(1);

  point_on_circle_2D = distance_to_inspect_point_ *
                           (point_2D - inspection_point_2D) /
                           ((point_2D - inspection_point_2D).norm()) +
                       inspection_point_2D;

  point_on_circle_3D(0) = point_on_circle_2D(0);
  point_on_circle_3D(1) = point_on_circle_2D(1);
  point_on_circle_3D(2) = point(2);

  return point_on_circle_3D;
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

  if (angle > M_PI)   angle = angle + 2*M_PI;

  return angle;
}