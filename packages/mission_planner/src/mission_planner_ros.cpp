#include "mission_planner_ros.hpp"

MissionPlannerRos::MissionPlannerRos(ros::NodeHandle _nh) : nh_(_nh) {
  // Subscribers
  cur_pose_sub_      = nh_.subscribe<geometry_msgs::PoseStamped>("/drone_1/ual/pose", 1, &MissionPlannerRos::uavPoseCallback, this); 
  cur_vel_sub_       = nh_.subscribe<geometry_msgs::TwistStamped>("/drone_1/ual/velocity", 1, &MissionPlannerRos::uavVelocityCallback, this); 

  // ros params
  safeGetParam(nh_, "horizon_length", param_.horizon_length);
  safeGetParam(nh_, "step_size", param_.step_size);
  // initialize mission planner
  mission_planner_ptr_ = std::make_unique<MissionPlanner>(param_);
}

MissionPlannerRos::~MissionPlannerRos() {}


// Callbacks
void MissionPlannerRos::uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    cur_pose_.pose.position.x = msg->pose.position.x;
    cur_pose_.pose.position.y = msg->pose.position.y;
    cur_pose_.pose.position.z = msg->pose.position.z; 

    cur_pose_.pose.orientation.x = msg->pose.orientation.x;
    cur_pose_.pose.orientation.y = msg->pose.orientation.y;
    cur_pose_.pose.orientation.z = msg->pose.orientation.z;
    cur_pose_.pose.orientation.w = msg->pose.orientation.w;

}

void MissionPlannerRos::uavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg){
    cur_vel_.twist.linear.x  = msg->twist.linear.x;
    cur_vel_.twist.linear.y  = msg->twist.linear.y;
    cur_vel_.twist.linear.z  = msg->twist.linear.z;

    cur_vel_.twist.angular.x     = msg->twist.angular.x;
    cur_vel_.twist.angular.y     = msg->twist.angular.y;
    cur_vel_.twist.angular.z     = msg->twist.angular.z;

}