#include "mission_planner_ros.hpp"

MissionPlannerRos::MissionPlannerRos(ros::NodeHandle _nh) : nh_(_nh) {
  // ros params
  safeGetParam(nh_, "horizon_length", param_.horizon_length);
  safeGetParam(nh_, "n_drones", param_.n_drones);
  safeGetParam(nh_, "step_size", param_.step_size);
  // initialize mission planner
  mission_planner_ptr_ = std::make_unique<MissionPlanner>(param_);

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

  // Services
  service_activate_planner = _nh.advertiseService("activate_planner", &MissionPlannerRos::activationPlannerServiceCallback, this);
}

MissionPlannerRos::~MissionPlannerRos() {}

// Callbacks
bool MissionPlannerRos::activationPlannerServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
  ROS_INFO("[%s]: Activation planner service called.", ros::this_node::getName().c_str());
  res.message = "Planning activated.";
  res.success = true;
  return true;
}

void MissionPlannerRos::replanCB(const ros::TimerEvent &e) {
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