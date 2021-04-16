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
  std::vector<state> reference_traj;
  std::vector<state> last_trajectory_;
  std::map<int, state> states_;
  MissionPlanner(const parameters _param);
  virtual ~MissionPlanner();
  virtual void appendGoal(const state &_goal){ goals_.push_back(std::move(_goal));}
  void clearGoals() { goals_.clear(); }
  std::vector<state> getGoals(){return goals_;}
  std::vector<state> getReferenceTrajectory(){return reference_traj;}
  void setPointToInspect(const Eigen::Vector3d &_point){  point_to_inspect_ = std::move(_point);}
  Eigen::Vector3d getPointToInspect(){ return point_to_inspect_;}
  void setDistanceToInspect(const float &_distance){ distance_to_inspect_point_ = _distance;}
  float getDistanceToInspect(){ return distance_to_inspect_point_;}
  void setRelativeAngle(const float &_angle){ relative_angle_ = _angle;}
  void plan();
  void setSolvedTrajectories(const std::vector<Eigen::Vector3d> &solved_trajectory, int _drone_id){solved_trajectories_[_drone_id]=solved_trajectory;}
  Eigen::Vector3d pointOnCircle(const Eigen::Vector3d point);
  
 protected:
  const parameters param_;
  const ACADO::Grid my_grid_;
  std::vector<state> goals_;
  Eigen::Vector3d point_to_inspect_ = Eigen::Vector3d::Zero();
  float distance_to_inspect_point_  = 3;
  float relative_angle_             = 0.4;
  std::map<int,std::vector<Eigen::Vector3d>> solved_trajectories_;
  int planner_state_ = PlannerStatus::FIRST_PLAN;

  /**
   * @brief Calculate a path from one point to another at cte velocity
   * 
   * @param initial point
   * @param final point
   * @return std::vector<state> path
   */
  std::vector<state> pathFromPointToAnother(const Eigen::Vector3d &initial, const Eigen::Vector3d &final);

    /**
   * @brief check formation poses
   * 
   * @return true if all poses are received
   * @return false 
   */
  
  bool hasPose(){return (states_.size() == param_.n_drones);}
  bool hasGoal(){return !goals_.empty();}
  bool waypointReached(const state &point, const state &waypoint) {return ((point.pos-waypoint.pos).norm() < REACH_TOL);}

 private:
  const float REACH_TOL = 1; //! tolerance to reach waypoints

  virtual std::vector<state> initialTrajectory(const state &_initial_pose){return pathFromPointToAnother(states_[param_.drone_id].pos, goals_[0].pos);}
  virtual std::vector<state> initialTrajectoryToInspect(){}
  virtual void optimalTrajectory(const std::vector<state> &initial_trajectory);
  virtual state nextGoal(){return goals_[0];}
  /**
   * @brief virtual function that makes the following checks
   * 
   * @return true if all checks are passed
   * @return false if any ot the check is not passed
   */
  virtual bool checks(){std::cout<<"check mission planner abstract"<<std::endl;}
  void initialOrientation(std::vector<state> &traj);
  void optimalOrientation(const std::vector<state> &traj_to_optimize);
  /**
   * @brief Utility function that checks if the drone is near the inspection zone
   * 
   */
  bool isInspectionZone(const Eigen::Vector3d &drone_pose);
};