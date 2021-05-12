#include "mission_planner.hpp"

#define M_PI           3.14159265358979323846

class MissionPlannerDurableLeader : public MissionPlanner {
 public:
  MissionPlannerDurableLeader(parameters params);
  ~MissionPlannerDurableLeader();
  void appendGoal(const state &_new_goal);

 protected:
  float rho;

 private:
  bool checks();
  float getTotalAngle(const float &_initial_angle, const float &_final_angle);
  bool isClockWise(const Eigen::Vector3d &_vector, const state &_state);
  std::vector<state> initialTrajectoryToInspect(const state &initial_pose);

};