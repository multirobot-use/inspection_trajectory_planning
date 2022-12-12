#include "mission_planner_inspection.hpp"

//! MissionPlannerInspectionFollower class
/*!
 * Class for follower drones that inherits of the class MissionPlannerInspection
 */

class MissionPlannerInspectionFollower : public MissionPlannerInspection {

 public:

  /**
   * @brief constructor of the class
   */
  MissionPlannerInspectionFollower(trajectory_planner::parameters _params, inspection_params _inspection_params);

  /**
   * @brief destructor of the class
   */
  ~MissionPlannerInspectionFollower();


 private:

  /**
   * @brief virtual function that makes the following checks
   *
   * @return true if all checks are passed
   * @return false if any ot the check is not passed
   */
  bool checks();

  /**
   * @brief returns an initial trajectory to inspect for the drone according to
   * the initial pose
   *
   * @param initial_pose initial pose of the drone
   * @return vector of states of the trajectory
   */
  virtual std::vector<trajectory_planner::state> initialTrajectory(
      const trajectory_planner::state &initial_pose) override;
  
  /**
   * @brief returns the inspection trajectory for the drone according to
   * the initial pose
   *
   * @param initial_pose initial pose of the drone
   * @return vector of states of the trajectory
   */
  virtual std::vector<state> inspectionTrajectory(const state &initial_pose) override;
};