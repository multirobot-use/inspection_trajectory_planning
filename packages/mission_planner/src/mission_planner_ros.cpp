#include "mission_planner_ros.hpp"

MissionPlannerRos::MissionPlannerRos(ros::NodeHandle _nh) : nh_(_nh) {
  // ros params
  safeGetParam(nh_, "horizon_length", param_.horizon_length);
  safeGetParam(nh_, "n_drones", param_.n_drones);
  safeGetParam(nh_, "step_size", param_.step_size);
  safeGetParam(nh_, "planning_rate", param_.planning_rate);
  safeGetParam(nh_, "drone_id", param_.drone_id);
  safeGetParam(nh_, "vel_max", param_.vel_max);
  safeGetParam(nh_, "acc_max", param_.acc_max);
  safeGetParam(nh_, "frame", param_.frame);
  safeGetParam(nh_, "drone_id", param_.drone_id);
  safeGetParam(nh_, "inspection_dist", param_.inspection_dist);

  // initialize mission planner
  mission_planner_ptr_ = std::make_unique<MissionPlannerDurableLeader>(param_);

  // Subscribers
  for (int drone = 1; drone <= param_.n_drones; drone++) {
    cur_pose_sub_[drone] = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/drone_" + std::to_string(drone) + "/ual/pose", 1,
        std::bind(&MissionPlannerRos::uavPoseCallback, this,
                  std::placeholders::_1, drone));
    cur_vel_sub_[drone] = nh_.subscribe<geometry_msgs::TwistStamped>(
        "/drone_" + std::to_string(drone) + "/ual/velocity", 1,
        std::bind(&MissionPlannerRos::uavVelocityCallback, this,
                  std::placeholders::_1, drone));
  }

  // markers config
  points_.header.frame_id = param_.frame;
  points_.header.stamp = ros::Time::now();
  points_.action = visualization_msgs::Marker::ADD;
  points_.pose.orientation.w =  1.0;
  points_.id = 0;
  points_.type = visualization_msgs::Marker::POINTS;
  points_.scale.x = 0.2;
  points_.scale.y = 0.2;
  points_.color.g = 1.0f;
  points_.color.a = 1.0;

  // create timer
  planTimer_ = nh_.createTimer(ros::Duration(param_.planning_rate),
                               &MissionPlannerRos::replanCB, this);
  pubVis_ = nh_.createTimer(ros::Duration(param_.planning_rate),
                               &MissionPlannerRos::pubVisCB, this);
  planTimer_.stop();
  pubVis_.start();

  // publishers
  points_pub_ = nh_.advertise<visualization_msgs::Marker>("points_to_inspect",1);
  pub_path_   = nh_.advertise<nav_msgs::Path>("solved_traj", 1);
  tracking_pub_   = nh_.advertise<nav_msgs::Path>("/drone_"+std::to_string(param_.drone_id)+"/upat_follower/follower/trajectory_to_follow", 1);
  tracking_pub_trajectory_   = nh_.advertise<trajectory_msgs::JointTrajectory>("/drone_"+std::to_string(param_.drone_id)+"/trajectory_follower_node/trajectory_to_follow", 1);

  // Services
  service_activate_planner = nh_.advertiseService(
      "activate_planner", &MissionPlannerRos::activationPlannerServiceCallback,
      this);
  service_waypoint = nh_.advertiseService(
      "add_waypoint", &MissionPlannerRos::addWaypointServiceCallback, this);
  service_point_to_inspect = nh_.advertiseService(
      "point_to_inspect", &MissionPlannerRos::pointToInspectServiceCallback,
      this);
  clear_waypoints = nh_.advertiseService(
      "clear_waypoints", &MissionPlannerRos::clearWaypointsServiceCallback,
      this);
  service_distance_to_inspect = nh_.advertiseService(
      "distance_to_inspect", &MissionPlannerRos::distanceToInspectServiceCallback,
      this);
  service_relative_angle = nh_.advertiseService(
      "change_relative_angle", &MissionPlannerRos::changeRelativeAngleServiceCallback,
      this);
}

MissionPlannerRos::~MissionPlannerRos() {}

// Callbacks
bool MissionPlannerRos::activationPlannerServiceCallback(
    std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  ROS_INFO("[%s]: Activation planner service called.",
           ros::this_node::getName().c_str());

  res.success = true;
  if (req.data == false) {
    ROS_INFO("[%s]: Planning deactivated.",
            ros::this_node::getName().c_str());
    res.message = "Planning deactivated.";
    planTimer_.stop();
  } else {
    ROS_INFO("[%s]: Planning activated.",
            ros::this_node::getName().c_str());
    res.message = "Planning activated.";
    planTimer_.start();
  }
  return true;
}

bool MissionPlannerRos::addWaypointServiceCallback(mission_planner::WaypointSrv::Request &req, mission_planner::WaypointSrv::Response &res){
  ROS_INFO("[%s]: Add waypoint service called.", ros::this_node::getName().c_str());
  state aux_goal;
  geometry_msgs::Point point;


  aux_goal.pos[0]   = req.waypoint.pose.pose.position.x;
  aux_goal.pos[1]   = req.waypoint.pose.pose.position.y;
  aux_goal.pos[2]   = req.waypoint.pose.pose.position.z;

  point.x = req.waypoint.pose.pose.position.x;
  point.y = req.waypoint.pose.pose.position.y;
  point.z = req.waypoint.pose.pose.position.z;

  aux_goal.vel[0]   = req.waypoint.twist.twist.linear.x;
  aux_goal.vel[1]   = req.waypoint.twist.twist.linear.y;
  aux_goal.vel[2]   = req.waypoint.twist.twist.linear.z;

  mission_planner_ptr_->appendGoal(aux_goal);
  points_.points.push_back(point); 

  res.success = true;
}

bool MissionPlannerRos::clearWaypointsServiceCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
  ROS_INFO("[%s]: Clear waypoints service called.", ros::this_node::getName().c_str());

  mission_planner_ptr_->clearGoals();
  points_.points.clear(); 
  
}

bool MissionPlannerRos::pointToInspectServiceCallback(mission_planner::PointToInspectSrv::Request &req, mission_planner::PointToInspectSrv::Response &res){
  ROS_INFO("[%s]: Point to inspect service called.", ros::this_node::getName().c_str());

  Eigen::Vector3d point;

  point[0] = req.point.x;
  point[1] = req.point.y;
  point[2] = req.point.z;
  
  mission_planner_ptr_->setPointToInspect(point);

  ROS_INFO("[%s]: Point to inspect changed successfully!   [X,  Y,  Z] = [%f,  %f,  %f]", ros::this_node::getName().c_str(), point[0], point[1], point[2]);
  res.success = true;

}

bool MissionPlannerRos::distanceToInspectServiceCallback(mission_planner::DistanceSrv::Request &req, mission_planner::DistanceSrv::Response &res){
  ROS_INFO("[%s]: Distance to inspect service called.", ros::this_node::getName().c_str());

  mission_planner_ptr_->setDistanceToInspect(req.distance);

  ROS_INFO("[%s]: Distance to inspection point changed successfully!  New distance:  %f meters", ros::this_node::getName().c_str(), req.distance);
  res.success = true;
}

bool MissionPlannerRos::changeRelativeAngleServiceCallback(mission_planner::AngleSrv::Request &req, mission_planner::AngleSrv::Response &res){
  ROS_INFO("[%s]: Change relative angle service called.", ros::this_node::getName().c_str());

  mission_planner_ptr_->setRelativeAngle(req.angle);

  ROS_INFO("[%s]: Relative angle changed successfully!  New angle:  %f radians", ros::this_node::getName().c_str(), req.angle);
  res.success = true;
}

void MissionPlannerRos::replanCB(const ros::TimerEvent &e) {
  ROS_INFO("Planning loop");
  mission_planner_ptr_->plan();
  publishTrajectoryJoint(tracking_pub_trajectory_, mission_planner_ptr_->last_trajectory_);
  // publishPath(tracking_pub_, mission_planner_ptr_->last_trajectory_);
  publishPath(pub_path_, mission_planner_ptr_->last_trajectory_);
}

void MissionPlannerRos::pubVisCB(const ros::TimerEvent &e) {
  ROS_INFO("Planning markers");
  points_pub_.publish(points_);
}

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

void MissionPlannerRos::publishPath(const ros::Publisher &pub_path, const std::vector<state> &trajectory){
  nav_msgs::Path path_to_publish;
  geometry_msgs::PoseStamped aux_pose;
  path_to_publish.header.frame_id = param_.frame;
  path_to_publish.header.stamp = ros::Time::now();
  for(const auto& state: trajectory){
    aux_pose.pose.position.x = state.pos(0);
    aux_pose.pose.position.y = state.pos(1);
    aux_pose.pose.position.z = state.pos(2);
    aux_pose.pose.orientation.x = state.orientation.x();
    aux_pose.pose.orientation.y = state.orientation.y();
    aux_pose.pose.orientation.z = state.orientation.z();
    aux_pose.pose.orientation.w = state.orientation.w();
    path_to_publish.poses.push_back(aux_pose);
  }
  try {
    pub_path.publish(path_to_publish);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_path.getTopic().c_str());
  }
}

void MissionPlannerRos::publishTrajectoryJoint(const ros::Publisher &pub_path, const std::vector<state> &trajectory){
  trajectory_msgs::JointTrajectory trajectory_to_follow;
  trajectory_msgs::JointTrajectoryPoint point_to_follow;
  for(auto &point : trajectory){
    point_to_follow.positions.clear();
    point_to_follow.velocities.clear();
    point_to_follow.positions.push_back(point.pos(0));
    point_to_follow.positions.push_back(point.pos(1));
    point_to_follow.positions.push_back(point.pos(2));
    point_to_follow.positions.push_back(point.orientation.x());
    point_to_follow.positions.push_back(point.orientation.y());
    point_to_follow.positions.push_back(point.orientation.z());
    point_to_follow.positions.push_back(point.orientation.w());

    point_to_follow.velocities.push_back(point.vel(0));
    point_to_follow.velocities.push_back(point.vel(1));
    point_to_follow.velocities.push_back(point.vel(2));
    trajectory_to_follow.points.push_back(point_to_follow);
  }
  try {
    pub_path.publish(trajectory_to_follow);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_path.getTopic().c_str());
  }
  
}

