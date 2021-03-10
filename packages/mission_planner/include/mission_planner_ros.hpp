#pragma once
#include "mission_planner.hpp"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class MissionPlannerRos {
  private:
    // Definitions
    ros::NodeHandle nh_;
    parameters param_;
    std::unique_ptr<MissionPlanner> mission_planner_ptr_;

    int                            drone_id_;  // From launch
    geometry_msgs::PoseStamped     cur_pose_;
    geometry_msgs::TwistStamped    cur_vel_;

    // Subscriptions
    ros::Subscriber               cur_pose_sub_;
    ros::Subscriber               cur_vel_sub_;

    // Callbacks prototypes
    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void uavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);

  public:
    MissionPlannerRos(ros::NodeHandle _nh);
    ~MissionPlannerRos();
};
