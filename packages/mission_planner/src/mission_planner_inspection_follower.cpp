#include <mission_planner_inspection_follower.hpp>

MissionPlannerInspectionFollower::MissionPlannerInspectionFollower(parameters params)
    : MissionPlanner(params) {}

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
    const state &_state) {
  const Eigen::Vector3d goal = rotateEig(states_[1].pos, relative_angle_);
  return pathFromPointToAnother(_state.pos, goal);
}

std::vector<state> MissionPlannerInspectionFollower::initialTrajectoryToInspect(
    const state &initial_pose) {
  if (hasSolvedTrajectories()) {
    state aux;
    std::vector<state> trajectory_to_optimize;
    trajectory_to_optimize.push_back(initial_pose);
    Eigen::Quaterniond rotation = eulerToQuat(0, 0, relative_angle_);
    Eigen::Matrix3d rotMat = rotation.toRotationMatrix();
    Eigen::Vector3d aux_point_to_inspect = point_to_inspect_;
    aux_point_to_inspect(2) = 0;
    for (int i = 1; i < param_.horizon_length; i++) {
      aux.pos = rotMat * (solved_trajectories_[param_.leader_id][i].pos -
                          aux_point_to_inspect) +
                aux_point_to_inspect;
      trajectory_to_optimize.push_back(std::move(aux));
    }

    return trajectory_to_optimize;
  } else {
    return initialTrajectory(initial_pose);
  }
}