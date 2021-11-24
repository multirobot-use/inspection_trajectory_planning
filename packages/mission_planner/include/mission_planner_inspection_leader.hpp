#include "mission_planner_inspection.hpp"

#define M_PI 3.14159265358979323846

//! MissionPlannerInspectionLeader class
/*!
 * Class for leader drones that inherits of the class MissionPlannerInspection
 */
class MissionPlannerInspectionLeader : public MissionPlannerInspection {

 public:
  /**
   * @brief constructor of the class
   */
  MissionPlannerInspectionLeader(trajectory_planner::parameters _params, inspection_params _inspection_params);

  /**
   * @brief destructor of the class
   */
  ~MissionPlannerInspectionLeader();

  /**
   * @brief pushes back a new goal on the vector of goal points to reach
   *
   * @param _goal new waypoint to add
   */
  void appendGoal(const state &_new_goal);


 protected:
  float rho;


 private:
  /**
   * @brief virtual function that makes the following checks
   *
   * @return true if all checks are passed
   * @return false if any ot the check is not passed
   */
  bool checks();

  /**
   * @brief gets the total angle to travel given an initial and a final angle
   *
   * @param _initial_angle initial angle
   * @param _final_angle final angle
   * @return total angle to travel
   */
  float getTotalAngle(const float &_initial_angle, const float &_final_angle);

  /**
   * @brief returns an initial trajectory to inspect for the drone according to
   * the initial pose
   *
   * @param initial_pose initial pose of the drone
   * @return vector of states of the trajectory
   */
  virtual std::vector<state> initialTrajectory(const state &initial_pose) override;
};