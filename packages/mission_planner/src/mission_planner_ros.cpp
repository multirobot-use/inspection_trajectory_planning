#include "mission_planner_ros.hpp"

MissionPlannerRos::MissionPlannerRos(ros::NodeHandle _nh) : nh_(_nh) {
  // ros params
  safeGetParam(nh_, "horizon_length", param_.horizon_length);
  safeGetParam(nh_, "n_drones", param_.n_drones);
  safeGetParam(nh_, "step_size", param_.step_size);
  // initialize mission planner
  mission_planner_ptr_ = std::make_unique<MissionPlanner>(param_);

  // Subscribers
  for (auto drone = 1; drone <= param_.n_drones; drone ++) {
    cur_pose_sub_[drone] = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/drone_" + std::to_string(drone) + "/ual/pose", 1,
        std::bind(&MissionPlannerRos::uavPoseCallback, this,
                  std::placeholders::_1, drone));
    cur_vel_sub_[drone] = nh_.subscribe<geometry_msgs::TwistStamped>(
        "/drone_" + std::to_string(drone) + "/ual/velocity", 1,
        std::bind(&MissionPlannerRos::uavVelocityCallback, this,
                  std::placeholders::_1, drone));
  }
}

MissionPlannerRos::~MissionPlannerRos() {}

// Callbacks
void MissionPlannerRos::uavPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg, int id) {
  cur_pose_[id].pose.position.x = msg->pose.position.x;
  cur_pose_[id].pose.position.y = msg->pose.position.y;
  cur_pose_[id].pose.position.z = msg->pose.position.z;

  cur_pose_[id].pose.orientation.x = msg->pose.orientation.x;
  cur_pose_[id].pose.orientation.y = msg->pose.orientation.y;
  cur_pose_[id].pose.orientation.z = msg->pose.orientation.z;
  cur_pose_[id].pose.orientation.w = msg->pose.orientation.w;
}

void MissionPlannerRos::uavVelocityCallback(
    const geometry_msgs::TwistStamped::ConstPtr &msg, int id) {
  cur_vel_[id].twist.linear.x = msg->twist.linear.x;
  cur_vel_[id].twist.linear.y = msg->twist.linear.y;
  cur_vel_[id].twist.linear.z = msg->twist.linear.z;

  cur_vel_[id].twist.angular.x = msg->twist.angular.x;
  cur_vel_[id].twist.angular.y = msg->twist.angular.y;
  cur_vel_[id].twist.angular.z = msg->twist.angular.z;
}