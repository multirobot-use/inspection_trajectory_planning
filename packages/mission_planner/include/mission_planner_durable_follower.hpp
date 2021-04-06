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

class MissionPlannerDurableFollower : public MissionPlanner {
    public:
        // Need to know which arguments are needed
        MissionPlannerDurableFollower(parameters params);
    
    private:
    //! FollowerDrone constructor
    MissionPlannerDurableFollower(ros::NodeHandle _nh);

    //! FollowerDrone destructor
    ~MissionPlannerDurableFollower();

    std::vector<state> initialTrajectory(const state &_state);
    void optimalTrajectory(const std::vector<state> &initial_trajectory);



};