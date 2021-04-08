#include "mission_planner.hpp"
class MissionPlannerDurableLeader : public MissionPlanner {
 public:
  MissionPlannerDurableLeader(parameters params);
  ~MissionPlannerDurableLeader();
  void appendGoal(const state &_new_goal);


 private:
  std::vector<state> initialTrajectory(const state &_state);
  void optimalTrajectory(const std::vector<state> &initial_trajectory);
};