#pragma once
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mission_planner/WaypointSrv.h>
#include <mission_planner/PointToInspectSrv.h>
#include <mission_planner/DistanceSrv.h>
#include <mission_planner/AngleSrv.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include "mission_planner_durable_leader.hpp"
#include "mission_planner_durable_follower.hpp"
#include <visualization_msgs/Marker.h>
#include <trajectory_msgs/JointTrajectory.h>

enum Colors { RED = 0, BLUE = 2, YELLOW = 3};


//!  MissionPlannerRos class.
/*!
  A class to handle the functionality of the mission planner on ROS.
*/

class MissionPlannerRos {
 public:
  //! MissionPlannerRos constructor
  MissionPlannerRos(ros::NodeHandle _nh, const bool leader);

  //! MissionPlannerRos destructor
  ~MissionPlannerRos();
 private:

  // Declarations
  ros::NodeHandle nh_;
  parameters param_;
  std::unique_ptr<MissionPlanner> mission_planner_ptr_;
  ros::Timer planTimer_;
  ros::Timer pubVis_;

  std::vector<geometry_msgs::Point> points_;

  // Subscriptions
  std::map<int, ros::Subscriber> cur_pose_sub_;
  std::map<int, ros::Subscriber> cur_vel_sub_;
  std::map<int, ros::Subscriber> solved_trajectories_sub_;
  std::map<int, ros::Subscriber> distance_to_inspection_point_sub_;
  std::map<int, ros::Subscriber> relative_angle_sub_;

  // Publishers
  ros::Publisher points_pub_;
  ros::Publisher points_trans_pub_;
  ros::Publisher pub_path_;
  ros::Publisher pub_ref_path_;
  ros::Publisher tracking_pub_;
  ros::Publisher tracking_pub_trajectory_;
  ros::Publisher sphere_pub_;
  ros::Publisher distance_to_inspection_point_pub_;
  ros::Publisher relative_angle_pub_;
  
  // Services
  ros::ServiceServer service_activate_planner;
  ros::ServiceServer service_waypoint;
  ros::ServiceServer clear_waypoints;
  ros::ServiceServer service_point_to_inspect;
  ros::ServiceServer service_distance_to_inspect;
  ros::ServiceServer service_relative_angle;

  //! Callback prototypes

  /**
   * @brief Callback for the solved trajectories from others
   * 
   * @param msg trajectory
   * @param id drone id
   */
  void solvedTrajCallback(const nav_msgs::Path::ConstPtr &msg, int id);

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
  
  /*! \brief Callback for inspection point service. It changes the inspection point
   *   \param req point to inspect (XYZ)
   *   \param res success
   */
  bool pointToInspectServiceCallback(mission_planner::PointToInspectSrv::Request &req, mission_planner::PointToInspectSrv::Response &res);
  
  /*! \brief Callback for the distance to inspect service. It changes the distance between the drones and the inspection point
   *   \param req distance (float)
   *   \param res success
   */
  bool distanceToInspectServiceCallback(mission_planner::DistanceSrv::Request &req, mission_planner::DistanceSrv::Response &res);

  /*! \brief Callback for the change the relative angle service. It changes the relative angles between the leader drone and the followers
   *   \param req angle (float)
   *   \param res success
   */
  bool changeRelativeAngleServiceCallback(mission_planner::AngleSrv::Request &req, mission_planner::AngleSrv::Response &res);

  /*! \brief Callback for timer that publishes rviz markers
  * \param TimerEvent structure passed to callback invoked by ros::Timer
  */
  void pubVisCB(const ros::TimerEvent &e);
  /*! \brief Callback for plan timer.
   *   \param TimerEvent structure passed to callback invoked by ros::Timer
   */
  void replanCB(const ros::TimerEvent &e);

  /*! \brief Method that publishes the distance to the inspection point with an increase
   *   \param pub_distance distance publisher
   *   \param distance current distance to inspection point
   *   \param increase value to increase the distance to inspection point
   **/
  void publishDistance(const ros::Publisher &pub_distance, const std_msgs::Float32 &distance, const std_msgs::Float32 &increase);

  /*! \brief Method that publishes the relative angle with an increase
   *   \param pub_distance relative angle publisher
   *   \param distance current relative angle
   *   \param increase value to increase the relative angle
   **/
  void publishAngle(const ros::Publisher &pub_angle, const std_msgs::Float32 &angle, const std_msgs::Float32 &increase);

  /*! \brief Callback for drone's pose
   *   \param msg drone's pose, geometry_msgs/PoseStamped
   *   \param id  identifier of the drone
   **/
  void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id);

  /*! \brief Callback for distance to inspection point (topic)
   *   \param distance desired distance to inspection point (absolute)
   *   \param id  identifier of the drone (not being used at this moment)
   **/
  void distanceToInspectionPointCallback(const std_msgs::Float32::ConstPtr &distance, int id);

  /*! \brief Callback for relative angle (topic)
   *   \param distance desired relative angle (absolute)
   *   \param id  identifier of the drone (not being used at this moment)
   **/
  void relativeAngleCallback(const std_msgs::Float32::ConstPtr &angle, int id);

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

  /*! \brief function to publish points on RViz
   *   \param pub_points publisher
   *   \param _points vector of points to publish
   *   \param color color of points
  */
  void publishPoints(const ros::Publisher &pub_points, const std::vector<geometry_msgs::Point> &_points, Colors color);

  /*! \brief function to publish the cylinder on RViz
   *   \param pub_sphere publisher
   *   \param color color of the cylinder
  */
  void publishSphere(const ros::Publisher &pub_sphere, const Colors &color);

  /*! \brief function to set the marker's color on RViz
   *   \param marker marker
   *   \param color color of the marker
  */
  void setMarkerColor(visualization_msgs::Marker &marker, const Colors &color);


};
