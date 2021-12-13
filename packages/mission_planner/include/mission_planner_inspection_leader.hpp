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
  bool stop_ = true;


 private:
  /**
   * @brief virtual function that makes the following checks
   *
   * @return true if all checks are passed
   * @return false if any ot the check is not passed
   */
  bool checks();

  /**
   * @brief function that calculates the velocity regarding the distance to the inspection point
   *
   * @return velocity
   */
  float calculateVel(const float &_distance);

  /**
   * @brief returns an initial trajectory to inspect for the drone according to
   * the initial pose
   *
   * @param initial_pose initial pose of the drone
   * @return vector of states of the trajectory
   */
  virtual std::vector<state> initialTrajectory(const state &initial_pose) override;

  /**
   * @brief returns the inspection trajectory for the drone according to
   * the initial pose
   *
   * @param initial_pose initial pose of the drone
   * @return vector of states of the trajectory
   */
  virtual std::vector<state> inspectionTrajectory(const state &initial_pose) override;
};