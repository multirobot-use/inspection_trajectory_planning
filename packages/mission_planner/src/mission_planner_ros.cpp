#include "mission_planner_ros.hpp"

MissionPlannerRos::MissionPlannerRos(ros::NodeHandle _nh) : nh_(_nh) {
  // ros params
  safeGetParam(nh_, "horizon_length", param_.horizon_length);
  safeGetParam(nh_, "n_drones", param_.n_drones);
  safeGetParam(nh_, "step_size", param_.step_size);
  safeGetParam(nh_, "planning_rate", param_.planning_rate);
  safeGetParam(nh_, "drone_id", param_.drone_id);

  // initialize mission planner
  mission_planner_ptr_ = std::make_unique<MissionPlannerDurable>(param_);

  // Subscribers
  for (auto drone = 1; drone <= param_.n_drones; drone++) {
    cur_pose_sub_[drone] = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/drone_" + std::to_string(drone) + "/ual/pose", 1,
        std::bind(&MissionPlannerRos::uavPoseCallback, this,
                  std::placeholders::_1, drone));
    cur_vel_sub_[drone] = nh_.subscribe<geometry_msgs::TwistStamped>(
        "/drone_" + std::to_string(drone) + "/ual/velocity", 1,
        std::bind(&MissionPlannerRos::uavVelocityCallback, this,
                  std::placeholders::_1, drone));
  }

  // create timer
  planTimer_ = nh_.createTimer(ros::Duration(param_.planning_rate),
                               &MissionPlannerRos::replanCB, this);
  planTimer_.stop();

  // Services
  service_activate_planner = nh_.advertiseService(
      "activate_planner", &MissionPlannerRos::activationPlannerServiceCallback,
      this);
  service_waypoint = nh_.advertiseService(
      "add_waypoint", &MissionPlannerRos::addWaypointServiceCallback, this);
  clear_waypoints = nh_.advertiseService(
      "clear_waypoints", &MissionPlannerRos::clearWaypointsServiceCallback,
      this);
}

MissionPlannerRos::~MissionPlannerRos() {}

// Callbacks
bool MissionPlannerRos::activationPlannerServiceCallback(
    std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  ROS_INFO("[%s]: Activation planner service called.",
           ros::this_node::getName().c_str());

  res.success = true;
  if (req.data == false) {
    res.message = "Planning deactivated.";
    planTimer_.stop();
  } else {
    res.message = "Planning activated.";
    planTimer_.start();
  }
  return true;
}

bool MissionPlannerRos::addWaypointServiceCallback(mission_planner::WaypointSrv::Request &req, mission_planner::WaypointSrv::Response &res){
  ROS_INFO("[%s]: Add waypoint service called.", ros::this_node::getName().c_str());

  state aux_goal;

  aux_goal.pos[0]   = req.waypoint.pose.pose.position.x;
  aux_goal.pos[1]   = req.waypoint.pose.pose.position.y;
  aux_goal.pos[2]   = req.waypoint.pose.pose.position.z;

  aux_goal.vel[0]   = req.waypoint.twist.twist.linear.x;
  aux_goal.vel[1]   = req.waypoint.twist.twist.linear.y;
  aux_goal.vel[2]   = req.waypoint.twist.twist.linear.z;

  mission_planner_ptr_->appendGoal(aux_goal);

  res.success = true;
}

bool MissionPlannerRos::clearWaypointsServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ROS_INFO("[%s]: Clear waypoints service called.", ros::this_node::getName().c_str());

  mission_planner_ptr_->clearGoals();

  return 1;
}

void MissionPlannerRos::replanCB(const ros::TimerEvent &e) {
  ROS_INFO("Planning loop");
  mission_planner_ptr_->plan();
}

void MissionPlannerRos::uavPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg, int id) {
  cur_state_[id].pos[0] = msg->pose.position.x;
  cur_state_[id].pos[1] = msg->pose.position.y;
  cur_state_[id].pos[2] = msg->pose.position.z;
}

void MissionPlannerRos::uavVelocityCallback(
    const geometry_msgs::TwistStamped::ConstPtr &msg, int id) {
  cur_state_[id].vel[0] = msg->twist.linear.x;
  cur_state_[id].vel[1] = msg->twist.linear.y;
  cur_state_[id].vel[2] = msg->twist.linear.z;
}