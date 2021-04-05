#pragma once

#include<mission_planner.hpp>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mission_planner/WaypointSrv.h>
#include <nav_msgs/Odometry.h>
#include "mission_planner_durable.hpp"
#include <visualization_msgs/Marker.h>

class FollowerDrone : public MissionPlanner {
    public:
        // Need to know which arguments are needed
        FollowerDrone(ros::NodeHandle _nh)
    
    private:
    ros::NodeHandle nh_;
    parameters param_;
    std::unique_ptr<MissionPlannerDurable> mission_planner_ptr_;
    ros::Timer planTimer_;
    ros::Timer pubVis_;

    // id_ of the drone (got from launch)
    int id_; 

    // Current state of the drone
    std::map<int, state> cur_state_;

    // Subscriptions
    ros::Subscriber leader_cur_pose_sub_;
    ros::Subscriber leader_cur_vel_sub_;

    ros::Subscriber follower_cur_pose_sub_;
    ros::Subscriber follower_cur_vel_sub_;

    // Publishers
    ros::Publisher points_pub_;

    // Services

    // markers
    visualization_msgs::Marker points_;


    // Callback functions

    /*! \brief Callback for drone's pose
    *   \param msg drone's pose, geometry_msgs/PoseStamped
    *   \param id  identifier of the drone
    **/
    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg, int id);

    /*! \brief Callback for drone's velocity
    *   \param msg drone's velocity, geometry_msgs/TwistStamped
    *   \param id  identifier of the drone
    **/
    void uavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg,
                            int id);


    //! FollowerDrone constructor
    FollowerDrone(ros::NodeHandle _nh);

    //! FollowerDrone destructor
    ~FollowerDrone();



};