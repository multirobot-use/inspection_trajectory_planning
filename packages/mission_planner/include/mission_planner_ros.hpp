#pragma once
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseArray.h>
#include <mission_planner/OperationModeSrv.h>
#include <mission_planner/AngleSrv.h>
#include <mission_planner/DistanceSrv.h>
#include <mission_planner/OrbitTimeSrv.h>
#include <mission_planner/PointToInspectSrv.h>
#include <mission_planner/WaypointSrv.h>
#include <mission_planner/Float32withHeader.h>
#include <mission_planner/AddWaypointByAngle.h>
#include <mission_planner/WaypointAngle.h>
#include <mission_planner/WaypointAngleArray.h>
#include <mission_planner/BoolWithHeader.h>
#include <uav_abstraction_layer/State.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <visualization_msgs/Marker.h>
#include "mission_planner_inspection_follower.hpp"
#include "mission_planner_inspection_leader.hpp"
#include "ros/ros.h"
#include <math.h>

#include <ros/callback_queue.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/common/transforms.h>

enum Colors { RED = 0, BLUE = 2, YELLOW = 3 };

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
  trajectory_planner::parameters param_;
  inspection_params inspection_params_;
  std::unique_ptr<MissionPlannerInspection> mission_planner_ptr_;
  ros::Timer planTimer_;
  ros::Timer clockTimer_;
  ros::Timer pubVis_;
  ros::Timer topicsTimer_;
  std::vector<geometry_msgs::Point> points_;

  // Subscriptions
  std::map<int, ros::Subscriber> cur_pose_sub_;
  std::map<int, ros::Subscriber> cur_state_sub_;
  std::map<int, ros::Subscriber> cur_vel_sub_;
  std::map<int, ros::Subscriber> solved_trajectories_sub_;
  std::map<int, ros::Subscriber> reference_trajectories_sub_;
  std::map<int, ros::Subscriber> inspection_distance_joy_sub_;
  std::map<int, ros::Subscriber> formation_angle_joy_sub_;
  std::map<int, ros::Subscriber> orbit_time_joy_sub_;
  std::map<int, ros::Subscriber> operation_mode_sub_;
  std::map<int, ros::Subscriber> planner_status_sub_;
  std::map<int, ros::Subscriber> waypoints_sub_;
  std::map<int, ros::Subscriber> waypoints_angle_sub_;

  std::map<int, ros::Subscriber> pcd_sub_;

  // Publishers
  ros::Publisher points_pub_;
  ros::Publisher points_trans_pub_;
  ros::Publisher waypoints_pub_;
  ros::Publisher waypoints_angle_pub_;
  ros::Publisher point_to_inspect_pub_;
  ros::Publisher pub_path_;
  ros::Publisher pub_ref_path_;
  ros::Publisher tracking_pub_;
  ros::Publisher tracking_pub_trajectory_;
  ros::Publisher sphere_pub_;
  ros::Publisher distance_pub_;
  ros::Publisher angle_pub_;
  ros::Publisher orbit_time_pub_;
  ros::Publisher mission_status_pub_;
  ros::Publisher corridor_pub_;
  ros::Publisher pub_point_cloud_;
  ros::Publisher formation_angle_pub_;
  ros::Publisher inspection_distance_pub_;
  ros::Publisher operation_mode_pub_;
  ros::Publisher planner_status_pub_;

  // Services
  ros::ServiceServer service_activate_planner;
  ros::ServiceServer service_waypoint;
  ros::ServiceServer service_waypoint_by_angle;
  ros::ServiceServer clear_waypoints;
  ros::ServiceServer service_point_to_inspect;
  ros::ServiceServer service_inspection_distance;
  ros::ServiceServer service_formation_angle;
  ros::ServiceServer service_operation_mode;
  ros::ServiceServer clear_first_waypoint;
  ros::ServiceServer service_orbit_time;

  // LiDAR and PointCloud necessary to instantiate: queues, spinners, buffers and listeners
  // Queues
  ros::CallbackQueue pcd_queue_1_;
  ros::CallbackQueue pcd_queue_2_;
  ros::CallbackQueue pcd_queue_3_;

  // Spinners (due to constructor, it cannot be grouped in a map)
  ros::AsyncSpinner async_spinner_1_{1, &pcd_queue_1_};
  ros::AsyncSpinner async_spinner_2_{1, &pcd_queue_2_};
  ros::AsyncSpinner async_spinner_3_{1, &pcd_queue_3_};
  
  // Buffers
  std::map<int, tf2_ros::Buffer> tfBuffer;

  // Listeners (due to constructor, it cannot be grouped in a map)
  tf2_ros::TransformListener tfListener_1_{tfBuffer[1], true};
  tf2_ros::TransformListener tfListener_2_{tfBuffer[2], true};
  tf2_ros::TransformListener tfListener_3_{tfBuffer[3], true};

  std::map<int, ros::SubscribeOptions> ops;

  /*! \brief Callback for point cloud data
   *   \param msg point cloud information
   **/
  void pcdCallback(const sensor_msgs::PointCloud2::ConstPtr &msg, const int id);

  //! Callback prototypes
  /**
   * @brief Callback for the solved trajectories from others
   *
   * @param msg trajectory
   * @param id drone id
   */
  void solvedTrajCallback(const nav_msgs::Path::ConstPtr &msg, int id);
  
  /**
   * @brief Callback for the reference trajectories from others
   *
   * @param msg trajectory
   * @param id drone id
   */
  void referenceTrajCallback(const nav_msgs::Path::ConstPtr &msg, int id);

  /**
   * @brief Callback for the leader's reference trajectory time
   *
   * @param msg time
   * @param id drone id
   */
  void leaderInitTrajTimeCallback(const mission_planner::Float32withHeader::ConstPtr &msg, int id);

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

  /*! \brief Callback for adding a waypoint by angle
   *   \param req new desired waypoint
   *   \param res success
   *   \return success
   */
  bool addWaypointByAngleServiceCallback(
        mission_planner::AddWaypointByAngle::Request &req,
        mission_planner::AddWaypointByAngle::Response &res);

  /*! \brief Callback for the clean waypoints service. It cleans all the
   * waypoints queued \param req request \param res success \return success
   */
  bool clearWaypointsServiceCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res);

  /*! \brief Callback for the clean waypoint service. It clean the first
   * waypoint queued \param req request \param res success \return success
   */
  bool clearFirstWaypointServiceCallback(std_srvs::Empty::Request &req,
                                         std_srvs::Empty::Response &res);

  /*! \brief Callback for inspection point service. It changes the inspection
   * point \param req point to inspect (XYZ) \param res success
   */
  bool pointToInspectServiceCallback(
      mission_planner::PointToInspectSrv::Request &req,
      mission_planner::PointToInspectSrv::Response &res);

  /*! \brief Callback for the distance to inspect service. It changes the
   * distance between the drones and the inspection point \param req distance
   * (float) \param res success
   */
  bool distanceToInspectServiceCallback(
      mission_planner::DistanceSrv::Request &req,
      mission_planner::DistanceSrv::Response &res);

  /*! \brief Callback for the orbit time service. It changes the
   * orbit time of the formation \param req time (float)
   * \param res success
   */
  bool orbitTimeServiceCallback(
      mission_planner::OrbitTimeSrv::Request &req,
      mission_planner::OrbitTimeSrv::Response &res);

  /*! \brief Callback for change the relative angle service. It changes the
   * relative angles between the leader drone and the followers \param req angle
   * (float) \param res success
   */
  bool changeRelativeAngleServiceCallback(
      mission_planner::AngleSrv::Request &req,
      mission_planner::AngleSrv::Response &res);

  /*! \brief Callback for the operation mode service. It changes the
   * operation mode \param req mode (int) \param res success
   */
  bool changeOperationModeServiceCallback(
      mission_planner::OperationModeSrv::Request &req,
      mission_planner::OperationModeSrv::Response &res);

  /*! \brief Callback for timer that publishes rviz markers
   * \param TimerEvent structure passed to callback invoked by ros::Timer
   */
  void pubVisCB(const ros::TimerEvent &e);

  /*! \brief Callback for plan timer.
   *   \param TimerEvent structure passed to callback invoked by ros::Timer
   */
  void replanCB(const ros::TimerEvent &e);

  /*! \brief Callback for topics of interest
   * \param TimerEvent structure passed to callback invoked by ros::Timer
   */
  void topicsCB(const ros::TimerEvent &e);

  /*! \brief Callback for update the current time.
   *   \param TimerEvent structure passed to callback invoked by ros::Timer
   */
  void clockCB(const ros::TimerEvent &e);

  /*! \brief Callback for drone's operation mode
   *   \param operation_mode leader's operation mode, std_msgs/Uint8
   *   \param id  identifier of the drone
   **/
  void operationModeCallback(const std_msgs::UInt8::ConstPtr &operation_mode, int id);

  /*! \brief Callback for waypoints
   *   \param waypoints list of waypoints, geometry_msgs/PoseArray
   *   \param id  identifier of the drone
   **/
  void waypointsCallback(const geometry_msgs::PoseArray::ConstPtr &waypoints, int id);

  /*! \brief Callback for waypoints expressed in angles
   *   \param waypoints list of waypoints, mission_planner/WaypointAngle
   *   \param id  identifier of the drone
   **/
  void waypointsAngleCallback(const mission_planner::WaypointAngleArray::ConstPtr &waypoints, int id);

  /*! \brief Callback for drone's planner status
   *   \param planner_status leader's planner status, std_msgs/Uint8
   *   \param id  identifier of the drone
   **/
  void plannerStatusCallback(const std_msgs::UInt8::ConstPtr &planner_status, int id);

  /*! \brief Callback for drone's pose
   *   \param msg drone's pose, geometry_msgs/PoseStamped
   *   \param id  identifier of the drone
   **/
  void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id);

  /*! \brief Callback for drone's state
   *   \param msg drone's state, uav_abstraction_layer/State
   *   \param id  identifier of the drone
   **/
  void uavStateCallback(
      const uav_abstraction_layer::State::ConstPtr &msg, int id);

  /*! \brief Callback for distance to inspection point (topic)
   *   \param distance desired distance to inspection point (absolute)
   *   \param id  identifier of the drone (not being used at this moment)
   **/
  void distanceToInspectionPointCallback(
      const std_msgs::Bool::ConstPtr &distance, int id);

  /*! \brief Callback for increasing/decreasing the orbit time (topic)
   *   \param time true if want to increase the orbit time, false if want to decrease
   *   \param id  identifier of the drone (not being used at this moment)
   **/
  void orbitTimeJoyCallback(
        const std_msgs::Bool::ConstPtr &time, int id);

  /*! \brief Callback for relative angle (topic)
   *   \param distance desired relative angle (absolute)
   *   \param id  identifier of the drone (not being used at this moment)
   **/
  void relativeAngleCallback(const std_msgs::Bool::ConstPtr &angle, int id);

  /*! \brief Callback for drone's velocity
   *   \param msg drone's velocity, geometry_msgs/TwistStamped
   *   \param id  identifier of the drone
   **/
  void uavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg,
                           int id);

  /*! \brief function to publish a trajectory
   */
  void publishPath(const ros::Publisher &pub_path,
                   const std::vector<state> &trajectory);

  /*! \brief function to publish the initial time for the leader's reference trajectory
   */
  void publishInitTrajTimeLeader(const ros::Publisher &pub_time_leader);

  /*! \brief function to publish the operation mode
   */
  void publishOperationMode(const ros::Publisher &pub_operation_mode);

  /*! \brief function to publish the planner status
   */
  void publishPlannerStatus(const ros::Publisher &pub_planner_status);

  /*! \brief function to publish trajectory to the solver
   */
  void publishTrajectoryJoint(const ros::Publisher &pub_path,
                              const std::vector<state> &trajectory);

  /*! \brief function to publish points on RViz
   *   \param pub_points publisher
   *   \param _points vector of points to publish
   *   \param color color of points
   */
  void publishPoints(const ros::Publisher &pub_points,
                     const std::vector<geometry_msgs::Point> &_points,
                     Colors color);

  /*! \brief function to publish waypoints to a topic
   *   \param pub_waypoints publisher
   *   \param _points vector of points to publish
   */
  void publishWaypoints(const ros::Publisher &pub_waypoints,
                        const geometry_msgs::PoseArray &_points);

  /*! \brief function to publish waypoints to a topic
   *   \param pub_waypoints publisher
   *   \param _points vector of points to publish
   */
  void publishWaypointsAngle(const ros::Publisher &pub_waypoints,
                             const mission_planner::WaypointAngleArray &_points);

  /*! \brief function to publish the cylinder on RViz
   *   \param pub_sphere publisher
   *   \param color color of the cylinder
   */
  void publishCylinder(const ros::Publisher &pub_cylinder, const Colors &color);

  /*! \brief function to publish the absolute distance to the inspection point
   *   \param pub_distance publisher
   */
  void publishDistance(const ros::Publisher &pub_distance);

  /*! \brief function to publish the inspection distance of a drone
   *   \param pub_distance publisher
   */
  void publishInspectionDistance(const ros::Publisher &pub_distance);

  /*! \brief function to publish the absolute relative angle between drones 
   *   \param pub_angle publisher
   */
  void publishRelativeAngle(const ros::Publisher &pub_angle);

  /*! \brief function to publish the orbit time 
   *   \param pub_orbit_time publisher
   */
  void publishOrbitTime(const ros::Publisher &pub_orbit_time);

  /*! \brief function to publish the current formation angle between the follower UAV and the leader UAV 
   *   \param pub_angle publisher
   */
  void publishFormationAngle(const ros::Publisher &pub_angle);

  /*! \brief function to publish the mission status 
   *   \param pub_status publisher
   */
  void publishMissionStatus(const ros::Publisher &pub_status);

  /*! \brief function to publish the point to inspect 
   *   \param pub_point_to_inspect publisher
   */
  void publishPointToInspect(const ros::Publisher &pub_point_to_inspect);

  /*! \brief function to set the marker's color on RViz
   *   \param marker marker
   *   \param color color of the marker
   */
  void setMarkerColor(visualization_msgs::Marker &marker, const Colors &color);
  
};