#pragma once

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <acado/acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <map>
#include "mission_planner_types.hpp"

//! MissionPlanner class
/*!
 * Abstract base class for mission planner. It cannot be instatiated
 */

enum PlannerStatus { FIRST_PLAN = 0, REPLANNED = 2 };
enum MissionStatus { GO_TO = 0, MISSION_ZONE = 1};

class MissionPlanner {
 public:
  std::vector<state> last_trajectory_;
  std::map<int, state> states_;
  MissionPlanner(const parameters _param);
  virtual ~MissionPlanner();
  void appendGoal(const state &);
  void clearGoals();
  void setPointToInspect(const Eigen::Vector3d &);
  void setDistanceToInspect(const float &);
  void setRelativeAngle(const float &);
  void plan();

  
 protected:
  const parameters param_;
  const ACADO::Grid my_grid_;
  std::vector<state> goals_;
  Eigen::Vector3d point_to_inspect_ = Eigen::Vector3d::Zero();
  float distance_to_inspect_point_  = 50;
  float relative_angle_             = 0.4;


 private:
  const float REACH_TOL = 1; //! tolerance to reach waypoints
  int planner_state_ = PlannerStatus::FIRST_PLAN;

  /*! \brief this function check if the planned trajectory has already reach the waypoint to inspect
  *   \return true if the last trajectory has reached the commanded waypoint
  */
  bool waypointReached();
  virtual std::vector<state> initialTrajectory(const state &_initial_pose);
  virtual void optimalTrajectory(const std::vector<state> &_initial_traj) = 0;
  void initialOrientation(std::vector<state> &traj);
  void optimalOrientation(const std::vector<state> &traj_to_optimize);

  bool hasGoal(){return !goals_.empty();}
  bool hasPose(){return (states_.count(param_.drone_id) != 0);}
  bool waypointReached(const state &point, const state &waypoint) {return ((point.pos-waypoint.pos).norm() < REACH_TOL);}
};