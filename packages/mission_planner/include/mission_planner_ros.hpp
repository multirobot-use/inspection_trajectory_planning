#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "mission_planner.hpp"
#include "ros/ros.h"

//!  MissionPlannerRos class.
/*!
  A class to handle the functionality of the mission planner on ROS.
*/

class MissionPlannerRos {
 private:
  // Declarations
  ros::NodeHandle nh_;
  parameters param_;
  std::unique_ptr<MissionPlanner> mission_planner_ptr_;
  ros::Timer planTimer_;

  //   Current pose variable
  std::map<int, geometry_msgs::PoseStamped> cur_pose_;

  //   Current velocity variable
  std::map<int, geometry_msgs::TwistStamped> cur_vel_;

  // Current state of the drone
  std::map<int, struct state>                        cur_state_;

  // Subscriptions
  std::map<int, ros::Subscriber> cur_pose_sub_;
  std::map<int, ros::Subscriber> cur_vel_sub_;

  //! Callback prototypes

  /*! \brief Callback for plan timer.
   *   \param TimerEvent structure passed to callback invoked by ros::Timer
   */
  void replanCB(const ros::TimerEvent &e);

  /*! \brief Callback for drone's pose
   *   \param msg drone's pose, geometry_msgs/PoseStamped
   *   \param id  identifier of the drone
   **/
  void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id);

  /*! \brief Callback for drone's velocity
   *   \param msg drone's velocity, geometry_msgs/TwistStamped
   *   \param id  identifier of the drone
   **/
  void uavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg,
                           int id);

 public:
  //! MissionPlannerRos constructor
  MissionPlannerRos(ros::NodeHandle _nh);

  //! MissionPlannerRos destructor
  ~MissionPlannerRos();
};
