#pragma once
#include "mission_planner.hpp"
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

class MissionPlannerRos {
  private:
    // Declarations
    ros::NodeHandle nh_;
    parameters param_;
    std::unique_ptr<MissionPlanner> mission_planner_ptr_;

    std::map<int, geometry_msgs::PoseStamped>     cur_pose_;
    std::map<int, geometry_msgs::TwistStamped>    cur_vel_;

    // Subscriptions
    std::map<int, ros::Subscriber>          cur_pose_sub_;
    std::map<int, ros::Subscriber>          cur_vel_sub_;

    // Callback prototypes
    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id);
    void uavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg, int id);

  public:
    MissionPlannerRos(ros::NodeHandle _nh);
    ~MissionPlannerRos();
};
