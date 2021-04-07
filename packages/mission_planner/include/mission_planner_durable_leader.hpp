#include "mission_planner.hpp"
class MissionPlannerDurableLeader : public MissionPlanner {
 public:
  MissionPlannerDurableLeader(parameters params);
  ~MissionPlannerDurableLeader();

 private:
  std::vector<state> initialTrajectory(const state &_state);
  Eigen::Vector3d pointOnCircle(const Eigen::Vector3d &_point);
  void optimalTrajectory(const std::vector<state> &initial_trajectory);
};