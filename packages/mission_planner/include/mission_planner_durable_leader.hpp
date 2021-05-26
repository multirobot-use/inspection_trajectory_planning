#include "mission_planner.hpp"

#define M_PI           3.14159265358979323846

//! MissionPlannerDurableLeader class
/*!
 * Class for leader drones that inherits of the class MissionPlanner
 */
class MissionPlannerDurableLeader : public MissionPlanner {
 public:
  /**
   * @brief constructor of the class
   */
  MissionPlannerDurableLeader(parameters params);

  /**
   * @brief destructor of the class
   */
  ~MissionPlannerDurableLeader();

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
   * @brief determines if the drone is travelling clockwise or anticlockwise
   * 
   * @param _vector vector of velocity of the drone
   * @param _state current position of the drone
   * @return true if clockwise
   * @return false if anticlockwise
   */
  bool isClockWise(const Eigen::Vector3d &_vector, const state &_state);

  /**
   * @brief returns an initial trajectory to inspect for the drone according to the initial pose
   * 
   * @param initial_pose initial pose of the drone
   * @return vector of states of the trajectory
   */
  std::vector<state> initialTrajectoryToInspect(const state &initial_pose);

};