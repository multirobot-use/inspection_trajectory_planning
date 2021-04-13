#include "mission_planner.hpp"
class MissionPlannerDurableLeader : public MissionPlanner {
 public:
  MissionPlannerDurableLeader(parameters params);
  ~MissionPlannerDurableLeader();
  void appendGoal(const state &_new_goal);


 private:
  bool checks();
  std::vector<state> initialTrajectory(const state &_initial_pose);
};