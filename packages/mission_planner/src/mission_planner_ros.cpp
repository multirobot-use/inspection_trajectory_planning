#include "mission_planner_ros.hpp"

MissionPlannerRos::MissionPlannerRos(ros::NodeHandle _nh) : nh_(_nh) {
  // Subscribers
  cur_pose_sub      = nh_.subscribe<geometry_msgs::PoseStamped>("/drone_1/ual/pose", 1, &MissionPlannerRos::uavPoseCallback, this); 
  cur_vel_sub       = nh_.subscribe<geometry_msgs::TwistStamped>("/drone_1/ual/velocity", 1, &MissionPlannerRos::uavVelocityCallback, this); 

}

MissionPlannerRos::~MissionPlannerRos() {}


// Callbacks
void MissionPlannerRos::uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    cur_pose.pose.position.x = msg->pose.position.x;
    cur_pose.pose.position.y = msg->pose.position.y;
    cur_pose.pose.position.z = msg->pose.position.z; 

    cur_pose.pose.orientation.x = msg->pose.orientation.x;
    cur_pose.pose.orientation.y = msg->pose.orientation.y;
    cur_pose.pose.orientation.z = msg->pose.orientation.z;
    cur_pose.pose.orientation.w = msg->pose.orientation.w;

}

void MissionPlannerRos::uavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg){
    cur_vel.twist.linear.x  = msg->twist.linear.x;
    cur_vel.twist.linear.y  = msg->twist.linear.y;
    cur_vel.twist.linear.z  = msg->twist.linear.z;

    cur_vel.twist.angular.x     = msg->twist.angular.x;
    cur_vel.twist.angular.y     = msg->twist.angular.y;
    cur_vel.twist.angular.z     = msg->twist.angular.z;

}