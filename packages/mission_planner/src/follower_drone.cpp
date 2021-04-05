#include <follower_drone.hpp>

FollowerDrone::FollowerDrone(ros::NodeHandle _nh){

    // Subscribers
    // Need to know the leader's position and its own position

    // Leader (0), drone_1
    leader_cur_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/drone_1/ual/pose", 1,
        std::bind(&MissionPlannerRos::uavPoseCallback, this,
                    std::placeholders::_1, 1));
    leader_cur_vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
        "/drone_1/ual/velocity", 1,
        std::bind(&MissionPlannerRos::uavVelocityCallback, this,
                    std::placeholders::_1, 1));

    // Follower (1), drone_x
    follower_cur_pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/drone_" + std::to_string(uav_id_) + "/ual/pose", 1,
        std::bind(&MissionPlannerRos::uavPoseCallback, this,
                  std::placeholders::_1, uav_id_));
    follower_cur_vel_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(
        "/drone_" + std::to_string(uav_id_) + "/ual/velocity", 1,
        std::bind(&MissionPlannerRos::uavVelocityCallback, this,
                  std::placeholders::_1, uav_id_));

}

FollowerDrone::~FollowerDrone(){}

void MissionPlannerRos::uavPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg, int id) {
    mission_planner_ptr_->states_[id].pos[0] = msg->pose.position.x;
    mission_planner_ptr_->states_[id].pos[1] = msg->pose.position.y;
    mission_planner_ptr_->states_[id].pos[2] = msg->pose.position.z;
}

void MissionPlannerRos::uavVelocityCallback(
    const geometry_msgs::TwistStamped::ConstPtr &msg, int id) {
    mission_planner_ptr_->states_[id].vel[0] = msg->twist.linear.x;
    mission_planner_ptr_->states_[id].vel[1] = msg->twist.linear.y;
    mission_planner_ptr_->states_[id].vel[2] = msg->twist.linear.z;
}
  