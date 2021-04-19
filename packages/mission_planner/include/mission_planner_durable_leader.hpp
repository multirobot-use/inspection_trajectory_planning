#include "mission_planner.hpp"

# define M_PI           3.14159265358979323846

class MissionPlannerDurableLeader : public MissionPlanner {
 public:
  MissionPlannerDurableLeader(parameters params);
  ~MissionPlannerDurableLeader();
  void appendGoal(const state &_new_goal);
 private:
  bool checks();
  float getAngle(const state &_state);
  bool isClockWise(const Eigen::Vector3d &_vector, float angle);
  std::vector<state> initialTrajectoryToInspect();

};