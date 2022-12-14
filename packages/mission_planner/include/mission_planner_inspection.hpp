#pragma once

#include <math.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <acado/acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <map>
// #include "log.h"
#include "mission_planner_types.hpp"
#include <trajectory_planner.hpp>
#include <uav_abstraction_layer/State.h>

typedef trajectory_planner::state state;

//! MissionPlannerInspection class
/*!
 * Abstract base class for mission planner. It cannot be instantiated
 */

class MissionPlannerInspection : public trajectory_planner::TrajectoryPlanner{

 public:

  /**
   * @brief constructor of the class
   */
  MissionPlannerInspection(const trajectory_planner::parameters _param, const inspection_params _inspection_params);

  /**
   * @brief destructor of the class
   */
  virtual ~MissionPlannerInspection();

  /**
   * @brief changes the desired point to inspect
   *
   * @param _point point to inspect
   */
  void setPointToInspect(const Eigen::Vector3d &_point) {
    point_to_inspect_ = std::move(_point);
  }

  /**
   * @brief gives the desired point to inspect
   *
   * @return point to inspect
   */
  Eigen::Vector3d getPointToInspect() { return point_to_inspect_; }

  /**
   * @brief pushes the waypoints expressed by angle and height
   *
   * @param _vector first component: angle; second component: height
   */
  void appendWaypointAngle(Eigen::Vector2d &_vector);

  /**
   * @brief pushes the waypoints expressed by angle and height
   *
   * @param _vector first component: angle; second component: height
   */
  std::vector<waypoint_angle_height> getWaypointsAngle() {
    return waypoints_angle_;
  }

  /**
   * @brief changes the desired distance to the inspection point
   *
   * @param _distance distance to the inspection point
   */
  void setDistanceToInspect(const float &_distance) {
    distance_to_inspect_point_ = _distance;
    if (distance_to_inspect_point_ > inspection_params_.max_inspection_dist) {
      distance_to_inspect_point_ = inspection_params_.max_inspection_dist;
    }
    else if (distance_to_inspect_point_ < inspection_params_.min_inspection_dist) {
      distance_to_inspect_point_ = inspection_params_.min_inspection_dist;
    }
  }

  /**
   * @brief changes the orbit time
   *
   * @param _time time of orbit the structure
   */
  void setOrbitTime(const float &_time) {
    if (orbit_time_ > inspection_params_.max_orbit_time) {
      orbit_time_ = inspection_params_.max_orbit_time;
    }
    else if (orbit_time_ < inspection_params_.min_orbit_time) {
      orbit_time_ = inspection_params_.min_orbit_time;
    }
  }

  /**
   * @brief increases/decreases the desired distance to the inspection point
   *
   * @param _distance true if increase, false if decrease
   */
  void incDistanceToInspect(const bool &_distance) {
    if (_distance)  setDistanceToInspect(distance_to_inspect_point_ + inspection_params_.inc_distance);
    else            setDistanceToInspect(distance_to_inspect_point_ - inspection_params_.inc_distance);
  }

  /**
   * @brief increases/decreases the orbit time
   *
   * @param _time true if increase, false if decrease
   */
  void incOrbitTime(const bool &_time) {
    if (_time)      setOrbitTime(orbit_time_ + inspection_params_.inc_orbit_time);
    else            setOrbitTime(orbit_time_ - inspection_params_.inc_orbit_time);
  }

  /**
   * @brief gives the distance to the inspection point
   *
   * @return distance to the inspection point
   */
  float getDistanceToInspect() { return distance_to_inspect_point_; }

  /**
   * @brief changes the desired relative angle of the drones
   *
   * @param _angle angle
   */
  void setRelativeAngle(const float &_angle) { 
    relative_angle_ = _angle;
    refreshRelativeAngle();
  }

  /**
   * @brief refreshes the formation angle
   *
   * @param _angle angle
   */
  void refreshRelativeAngle() {
    if (relative_angle_ > maximumFormationAngle())        relative_angle_ = maximumFormationAngle();
    else if (relative_angle_ < minimumFormationAngle())   relative_angle_ = minimumFormationAngle();
  }

  /**
   * @brief gives the relative angle
   *
   * @return relative angle
   */
  float getRelativeAngle() { return relative_angle_; }

  /**
   * @brief gives the relative angle
   *
   * @return relative angle
   */
  float getOrbitTime() { return orbit_time_; }

  /**
   * @brief gives the current formation angle of a UAV
   *
   * @return formation angle
   */
  float getFormationAngle(const int &_id) { return calculateFormationAngle(_id); }

  /**
   * @brief gives the current inspection distance of a UAV
   *
   * @return inspection distance
   */
  float getInspectionDistance(const int &_id) { return calculateInspectionDistance(_id); }

  /**
   * @brief gives the mission status
   *
   * @return mission status: true if activated, false if not activated
   */
  float getMissionStatus() { return mission_status_; }

  /**
   * @brief sets the current time (obtained from ROS::Time)
   */
  void setCurrentTime(float time_) {current_time_ = time_;}

  /**
   * @brief sets the mission status
   *
   * @param _status mission status
   */
  void setMissionStatus(const bool &_status) { mission_status_ = _status; }

  /**
   * @brief returns the status of the drone
   *
   * @param _drone_id identifier of the drone
   * @return state value
   */
  int getDroneStatus(const int _drone_id){ return drone_state_[param_.drone_id]; }

  /**
   * @brief sets the status of the drone
   *
   * @param _drone_id identifier of the drone
   * @param _state value
   */
  void setDroneStatus(const int _drone_id, const int _state){
    drone_state_[param_.drone_id] = _state; }

  /**
   * @brief changes the operation mode of the formation
   *
   * @param _mode mode
   */
  void setOperationMode(const uint &_mode) { operation_mode_ = _mode; }

  /**
   * @brief increases/decreases the relative angle of the drones
   *
   * @param _angle true if increase, false if decrease
   */
  void incRelativeAngle(const bool &_angle) {
    if (_angle)  setRelativeAngle(relative_angle_ + inspection_params_.inc_angle);
    else         setRelativeAngle(relative_angle_ - inspection_params_.inc_angle);
  }

  /**
   * @brief fits a given point to the cylinder/circle where the drones are
   * moving around
   *
   * @param point desired point to fit on the cylinder/circle
   * @return point on the cylinder/circle
   */
  Eigen::Vector3d pointOnCircle(const Eigen::Vector3d _point);

  /**
   * @brief fits a given point to the cylinder/circle where the drones are
   * moving around
   *
   * @param _vector first component: angle (rad); second component: height (m)
   * @return point on the cylinder/circle (3D)
   */
  Eigen::Vector3d pointOnCircleAngle(const Eigen::Vector2d _vector);

  /**
   * @brief gets the angle of a given point based on the inspection point
   *
   * @param point desired point to infer its angle
   * @return angle (between 0 and PI)
   */
  float getPointAngle(const Eigen::Vector3d &_point);

  /**
   * @brief function that returns if path to describe between two point is clockwise or anticlockwise
   *
   * @param _point1 point to start
   * @param _point2 point to finish
   * 
   * @return true if clockwise
   * @return false if anticlockwise
   */
  bool isClockwise(const Eigen::Vector3d &_point1, const Eigen::Vector3d &_point2);

  /**
   * @brief sets a new bunch of waypoints
   */
  void setGoals(const std::vector<trajectory_planner::state> &_waypoints) {
    goals_.clear();
    for (auto &waypoint : _waypoints) {
      goals_.push_back(waypoint);
    }
  }

  /**
   * @brief clears all the waypoints on the queue
   */
  void clearWaypointsAngle() { waypoints_angle_.clear(); }

  /**
   * @brief function that calculates the velocity regarding the distance to the inspection point
   *
   * @return velocity
   */
  float calculateVel(const float &_distance);


 protected:
  bool mission_status_ = false;
  state last_goal_;
  std::map<int, int> drone_state_;
  Eigen::Vector3d point_to_inspect_ = Eigen::Vector3d::Zero();
  float distance_to_inspect_point_ = 6.5;
  float relative_angle_ = 0.7;
  float orbit_time_ = 90;
  std::map<int, float> inspection_distance_;
  std::map<int, float> formation_angle_;
  std::vector<waypoint_angle_height> waypoints_angle_;
  inspection_params inspection_params_;

  /**
   * @brief refreshes the value of the goal points
   */
  void refreshGoals() {
    for (auto &goal : goals_) {
      goal.pos = pointOnCircle(goal.pos);
    }
  }

  /**
   * @brief refreshes the value of the goal points
   */
  void refreshGoalsExpressedInAngle() {
    Eigen::Vector2d waypoint_aux;
    int it = 0;
    for (auto &goal : goals_) {
      waypoint_aux(0) = waypoints_angle_[it].angle;
      waypoint_aux(1) = waypoints_angle_[it].height;
      goal.pos = pointOnCircleAngle(waypoint_aux);

      it = it + 1;
    }
  }

  /**
   * @brief gets the total angle to travel given an initial and a final angle
   *
   * @param _initial_angle initial angle
   * @param _final_angle final angle
   * @return total angle to travel
   */
  float getTotalAngle(const float &_initial_angle, const float &_final_angle);

  /**
   * @brief calculates the formation angle that the leader UAV has with the _id follower UAV
   *
   * @param _id id of the follower UAV
   * @return formation angle
   */
  float calculateFormationAngle(const int &_id);

  /**
   * @brief calculates the inspection distance of the _id UAV
   *
   * @param _id id of the follower UAV
   * @return inspection distance
   */
  float calculateInspectionDistance(const int &_id);

  /**
   * @brief calculates the minimum formation angle advisable
   *
   * @return minimum value of formation angle
   */
  float minimumFormationAngle();

  /**
   * @brief calculates the maximum formation angle advisable
   *
   * @return maximum value of formation angle
   */
  float maximumFormationAngle();

 private:
  /**
   * @brief returns an initial trajectory to inspect for the drone according to
   * the initial pose
   *
   * @param initial_pose initial pose of the drone
   * @return vector of states of the trajectory
   */
  virtual std::vector<state> initialTrajectory(
      const state &initial_pose) = 0;

  /**
   * @brief returns the inspection trajectory for the drone according to
   * the initial pose
   *
   * @param initial_pose initial pose of the drone
   * @return vector of states of the trajectory
   */
  virtual std::vector<state> inspectionTrajectory(
      const state &initial_pose) = 0;

  /**
   * @brief virtual function that makes the following checks
   *
   * @return true if all checks are passed
   * @return false if any ot the check is not passed
   */
  virtual bool checks() {
    std::cout << "check mission planner abstract" << std::endl;
  }

  /**
   * @brief check if the formation has to inspect
   *
   * @return true if the formation has to inspect
   * @return false if the formation does not have to inspect
   */
  virtual bool inspecting();

  /**
   * @brief gives an initial orientation according to a trajectory given
   *
   * @param traj trajectory
   */
  virtual void initialOrientation(std::vector<state> &traj);


};