#pragma once
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mission_planner/WaypointSrv.h>
#include <mission_planner/PointToInspectSrv.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include "mission_planner_durable_leader.hpp"
#include "mission_planner_durable_follower.hpp"
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/JointTrajectory.h>


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
  ros::Timer pubVis_;

  // Subscriptions
  std::map<int, ros::Subscriber> cur_pose_sub_;
  std::map<int, ros::Subscriber> cur_vel_sub_;

  // Publishers
  ros::Publisher points_pub_;
  ros::Publisher pub_path_;
  ros::Publisher tracking_pub_;
  ros::Publisher tracking_pub_trajectory_;
  
  // Services
  ros::ServiceServer service_activate_planner;
  ros::ServiceServer service_waypoint;
  ros::ServiceServer clear_waypoints;
  ros::ServiceServer service_point_to_inspect;

  // markers
  visualization_msgs::Marker points_;


  //! Callback prototypes

  /*! \brief Callback for the activate planner service.
   *   \param req structure of the request message
   *   \param res structure of the response message
   *   \return planning_activated
   */
  bool activationPlannerServiceCallback(std_srvs::SetBool::Request &req,
                                        std_srvs::SetBool::Response &res);

  /*! \brief Callback for the waypoint service. It adds a desired waypoint
   *   \param req new desired waypoint
   *   \param res success
   *   \return success
   */
  bool addWaypointServiceCallback(mission_planner::WaypointSrv::Request &req,
                                  mission_planner::WaypointSrv::Response &res);

  /*! \brief Callback for the clean waypoints service. It cleans all the
   * waypoints queued \param req request \param res success \return success
   */
  bool clearWaypointsServiceCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res);
  
  /*! \brief Callback for the waypoint service. It adds a desired waypoint
   *   \param req point to inspect (XYZ)
   *   \param res success
   */
  bool pointToInspectServiceCallback(mission_planner::PointToInspectSrv::Request &req, mission_planner::PointToInspectSrv::Response &res);
  
  /*! \brief Callback for timer that publishes rviz markers
  * \param TimerEvent structure passed to callback invoked by ros::Timer
  */
  void pubVisCB(const ros::TimerEvent &e);
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

  /*! \brief function to publish last solved trajectory
  */
  void publishPath(const ros::Publisher &pub_path, const std::vector<state> &trajectory);

  /*! \brief function to publish trajectory to the solver
  */
  void publishTrajectoryJoint(const ros::Publisher &pub_path, const std::vector<state> &trajectory);

 public:
  //! MissionPlannerRos constructor
  MissionPlannerRos(ros::NodeHandle _nh);

  //! MissionPlannerRos destructor
  ~MissionPlannerRos();
};
