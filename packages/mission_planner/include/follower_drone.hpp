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

    // Current state of the drone
    std::map<int, state> cur_state_;

    // Subscriptions
    std::map<int, ros::Subscriber> cur_pose_sub_;
    std::map<int, ros::Subscriber> cur_vel_sub_;

    // Publishers
    ros::Publisher points_pub_;
    ros::Publisher pub_path_;
    ros::Publisher tracking_pub_;

    // Services
    ros::ServiceServer service_activate_planner;
    ros::ServiceServer service_waypoint;
    ros::ServiceServer clear_waypoints;

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