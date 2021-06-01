#include "mission_planner.hpp"

//! MissionPlannerDurableFollower class
/*!
 * Class for follower drones that inherits of the class MissionPlanner
 */

class MissionPlannerDurableFollower : public MissionPlanner {
 public:
  /**
   * @brief constructor of the class
   */
  MissionPlannerDurableFollower(parameters params);

  /**
   * @brief destructor of the class
   */
  ~MissionPlannerDurableFollower();

 private:
  /**
   * @brief returns an initial straight trajectory for the drone according to
   * the initial pose
   *
   * @param initial_pose initial pose of the drone
   * @return vector of states of the trajectory
   */
  std::vector<state> initialTrajectory(const state &_state) override;

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
  std::vector<state> initialTrajectoryToInspect(
      const state &initial_pose) override;
};