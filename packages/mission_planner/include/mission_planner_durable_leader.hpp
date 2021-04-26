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
  float getPassAngle(const state &_initial_point, const state &_final_point);
  int getSectionSteps(const state &_initial_point, const state &_final_point, const bool &_clockwise);
  state getSectionPoint(const state &_state, const bool &_clockwise, const float &_angle_pass, const int &_total_section_steps, const int &_iteration);
  bool isClockWise(const Eigen::Vector3d &_vector, const state &_state);
  std::vector<state> initialTrajectoryToInspect();

};