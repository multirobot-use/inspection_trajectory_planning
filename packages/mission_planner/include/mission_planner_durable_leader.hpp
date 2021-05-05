#include "mission_planner.hpp"

#define M_PI           3.14159265358979323846

class MissionPlannerDurableLeader : public MissionPlanner {
 public:
  MissionPlannerDurableLeader(parameters params);
  ~MissionPlannerDurableLeader();
  void appendGoal(const state &_new_goal);
 private:
  bool checks();
  float velmaxToPolar(const float &_total_angle);
  float getTotalAngleOfSection(const float &_initial_angle, const float &_final_angle, const bool &_clockwise);
  int getSectionSteps(const state &_initial_point, const state &_final_point, const bool &_clockwise);
  state calcNextPoint(const state &_state, const bool &_clockwise, const int &_goal);
  bool isClockWise(const Eigen::Vector3d &_vector, const state &_state);
  std::vector<state> initialTrajectoryToInspect(const state &initial_pose);

};