#include "mission_planner_ros.hpp"

MissionPlannerRos::MissionPlannerRos(ros::NodeHandle _nh, const bool leader)
    : nh_(_nh) {
  // ros params
  trajectory_planner::safeGetParam(nh_, "horizon_length", param_.horizon_length);
  trajectory_planner::safeGetParam(nh_, "n_drones", param_.n_drones);
  trajectory_planner::safeGetParam(nh_, "step_size", param_.step_size);
  trajectory_planner::safeGetParam(nh_, "planning_rate", param_.planning_rate);
  trajectory_planner::safeGetParam(nh_, "drone_id", param_.drone_id);
  trajectory_planner::safeGetParam(nh_, "vel_max", param_.vel_max);
  trajectory_planner::safeGetParam(nh_, "acc_max", param_.acc_max);
  trajectory_planner::safeGetParam(nh_, "frame", param_.frame);
  trajectory_planner::safeGetParam(nh_, "drone_id", param_.drone_id);
  trajectory_planner::safeGetParam(nh_, "inspection_dist", inspection_params_.inspection_dist);
  trajectory_planner::safeGetParam(nh_, "topics_rate", param_.topics_rate);
  trajectory_planner::safeGetParam(nh_, "visualization_rate", param_.visualization_rate);
  trajectory_planner::safeGetParam(nh_, "leader_id", inspection_params_.leader_id);
  trajectory_planner::safeGetParam(nh_, "inc_distance", inspection_params_.inc_distance);
  trajectory_planner::safeGetParam(nh_, "inc_angle", inspection_params_.inc_angle);

  // initialize mission planner
  if (leader) {
    ROS_INFO("I'm a leader");
    mission_planner_ptr_ =
        std::make_unique<MissionPlannerInspectionLeader>(param_, inspection_params_);
  } else {
    ROS_INFO("I'm a follower");
    mission_planner_ptr_ =
        std::make_unique<MissionPlannerInspectionFollower>(param_, inspection_params_);
  }

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

    distance_to_inspection_point_sub_[drone] = nh_.subscribe<std_msgs::Bool>(
        "/drone_" + std::to_string(drone) +
            "/mission_planner_ros/distance_to_inspection_point",
        1,
        std::bind(&MissionPlannerRos::distanceToInspectionPointCallback, this,
                  std::placeholders::_1, drone));

    relative_angle_sub_[drone] = nh_.subscribe<std_msgs::Bool>(
        "/drone_" + std::to_string(drone) +
            "/mission_planner_ros/relative_angle",
        1,
        std::bind(&MissionPlannerRos::relativeAngleCallback, this,
                  std::placeholders::_1, drone));

    if (drone != param_.drone_id) {
      solved_trajectories_sub_[drone] = nh_.subscribe<nav_msgs::Path>(
          "/drone_" + std::to_string(drone) +
              "/mission_planner_ros/solved_traj",
          1,
          std::bind(&MissionPlannerRos::solvedTrajCallback, this,
                    std::placeholders::_1, drone));
    }
  }

  // create timer
  planTimer_ = nh_.createTimer(ros::Duration(param_.planning_rate),
                               &MissionPlannerRos::replanCB, this);
  pubVis_ = nh_.createTimer(ros::Duration(param_.visualization_rate),
                            &MissionPlannerRos::pubVisCB, this);
  pubTopics_ = nh_.createTimer(ros::Duration(param_.topics_rate),
                            &MissionPlannerRos::pubTopicsCB, this);
  planTimer_.stop();
  pubVis_.start();
  pubTopics_.start();

  // publishers
  points_pub_ =
      nh_.advertise<visualization_msgs::Marker>("points_to_inspect", 1);
  points_trans_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "points_to_inspect_transformed", 1);
  sphere_pub_ =
      nh_.advertise<visualization_msgs::Marker>("inspection_sphere", 1);
  pub_path_ = nh_.advertise<nav_msgs::Path>("solved_traj", 1);
  pub_ref_path_ = nh_.advertise<nav_msgs::Path>("ref_traj", 1);
  tracking_pub_ = nh_.advertise<nav_msgs::Path>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/upat_follower/follower/trajectory_to_follow",
      1);
  tracking_pub_trajectory_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/trajectory_follower_node/trajectory_to_follow",
      1);
  distance_pub_ = nh_.advertise<mission_planner::Float32withHeader>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/absolute_distance_to_inspect",
      1);
  angle_pub_ = nh_.advertise<mission_planner::Float32withHeader>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/absolute_relative_angle",
      1);
  mission_status_pub_ = nh_.advertise<mission_planner::BoolWithHeader>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/mission_status",
      1);

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
      "distance_to_inspect",
      &MissionPlannerRos::distanceToInspectServiceCallback, this);
  service_relative_angle = nh_.advertiseService(
      "change_relative_angle",
      &MissionPlannerRos::changeRelativeAngleServiceCallback, this);
}

MissionPlannerRos::~MissionPlannerRos() {}

// Callbacks
bool MissionPlannerRos::activationPlannerServiceCallback(
    std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  ROS_INFO("[%s]: Activation planner service called.",
           ros::this_node::getName().c_str());

  res.success = true;

  if (req.data == false) {
    ROS_INFO("[%s]: Planning deactivated.", ros::this_node::getName().c_str());
    res.message = "Planning deactivated.";
    planTimer_.stop();
    mission_planner_ptr_ -> setMissionStatus(false);
  } else {
    ROS_INFO("[%s]: Planning activated.", ros::this_node::getName().c_str());
    res.message = "Planning activated.";
    planTimer_.start();
    mission_planner_ptr_ -> setMissionStatus(true);
  }
  return true;
}

bool MissionPlannerRos::addWaypointServiceCallback(
    mission_planner::WaypointSrv::Request &req,
    mission_planner::WaypointSrv::Response &res) {
  ROS_INFO("[%s]: Add waypoint service called.",
           ros::this_node::getName().c_str());
  geometry_msgs::Point point;
  point.x = req.waypoint.pose.pose.position.x;
  point.y = req.waypoint.pose.pose.position.y;
  point.z = req.waypoint.pose.pose.position.z;
  points_.push_back(std::move(point));

  state state_req;

  state_req.pos(0) = req.waypoint.pose.pose.position.x;
  state_req.pos(1) = req.waypoint.pose.pose.position.y;
  state_req.pos(2) = req.waypoint.pose.pose.position.z;
  state_req.vel[0] = req.waypoint.twist.twist.linear.x;
  state_req.vel[1] = req.waypoint.twist.twist.linear.y;
  state_req.vel[2] = req.waypoint.twist.twist.linear.z;

  mission_planner_ptr_->appendGoal(state_req);

  res.success = true;
}

bool MissionPlannerRos::clearWaypointsServiceCallback(
    std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("[%s]: Clear waypoints service called.",
           ros::this_node::getName().c_str());

  mission_planner_ptr_->clearGoals();
}

bool MissionPlannerRos::pointToInspectServiceCallback(
    mission_planner::PointToInspectSrv::Request &req,
    mission_planner::PointToInspectSrv::Response &res) {
  ROS_INFO("[%s]: Point to inspect service called.",
           ros::this_node::getName().c_str());

  Eigen::Vector3d point;

  point[0] = req.point.x;
  point[1] = req.point.y;
  point[2] = req.point.z;

  mission_planner_ptr_->setPointToInspect(point);

  ROS_INFO(
      "[%s]: Point to inspect changed successfully!  [X,  Y,  Z] = [%f,  %f,  "
      "%f]",
      ros::this_node::getName().c_str(), point[0], point[1], point[2]);
  res.success = true;
}

bool MissionPlannerRos::distanceToInspectServiceCallback(
    mission_planner::DistanceSrv::Request &req,
    mission_planner::DistanceSrv::Response &res) {
  ROS_INFO("[%s]: Distance to inspect service called.",
           ros::this_node::getName().c_str());

  mission_planner_ptr_->setDistanceToInspect(req.distance);

  ROS_INFO(
      "[%s]: Distance to inspection point changed successfully!  New distance: "
      " %f meters",
      ros::this_node::getName().c_str(), req.distance);
  res.success = true;
}

bool MissionPlannerRos::changeRelativeAngleServiceCallback(
    mission_planner::AngleSrv::Request &req,
    mission_planner::AngleSrv::Response &res) {
  ROS_INFO("[%s]: Change relative angle service called.",
           ros::this_node::getName().c_str());

  mission_planner_ptr_->setRelativeAngle(req.angle);

  ROS_INFO("[%s]: Relative angle changed successfully!  New angle:  %f radians",
           ros::this_node::getName().c_str(), req.angle);
  res.success = true;
}

void MissionPlannerRos::replanCB(const ros::TimerEvent &e) {
  if (mission_planner_ptr_->getStatus() != trajectory_planner::PlannerStatus::FIRST_PLAN) {
    publishTrajectoryJoint(tracking_pub_trajectory_,
                           mission_planner_ptr_->getLastTrajectory());
    // publishPath(tracking_pub_, mission_planner_ptr_->last_trajectory_);
    publishPath(pub_path_, mission_planner_ptr_->getLastTrajectory());
    publishPath(pub_ref_path_, mission_planner_ptr_->getReferenceTrajectory());
  }
  mission_planner_ptr_->plan();
}

void MissionPlannerRos::pubVisCB(const ros::TimerEvent &e) {
  // publish commanded waypoint
  publishPoints(points_pub_, points_, Colors::RED);

  // publish transformed waypoints
  std::vector<state> goals = mission_planner_ptr_->getGoals();

  geometry_msgs::Point point;
  std::vector<geometry_msgs::Point> points;

  for (auto const &goal : goals) {
    point.x = goal.pos(0);
    point.y = goal.pos(1);
    point.z = goal.pos(2);
    points.push_back(point);
  }

  publishPoints(points_trans_pub_, points, Colors::BLUE);
  publishSphere(sphere_pub_, Colors::YELLOW);
}

void MissionPlannerRos::pubTopicsCB(const ros::TimerEvent &e) {
  publishDistance(distance_pub_);
  publishRelativeAngle(angle_pub_);
  publishMissionStatus(mission_status_pub_);
}

void MissionPlannerRos::distanceToInspectionPointCallback(
    const std_msgs::Bool::ConstPtr &distance, int id) {

  mission_planner_ptr_ -> incDistanceToInspect(distance->data);
}

void MissionPlannerRos::relativeAngleCallback(
    const std_msgs::Bool::ConstPtr &angle, int id) {

  mission_planner_ptr_->incRelativeAngle(angle->data);
}

void MissionPlannerRos::solvedTrajCallback(const nav_msgs::Path::ConstPtr &msg,
                                           int id) {
  state aux_state;
  std::vector<state> path;
  for (auto pose : msg->poses) {
    aux_state.pos(0) = pose.pose.position.x;
    aux_state.pos(1) = pose.pose.position.y;
    aux_state.pos(2) = pose.pose.position.z;
    path.push_back(aux_state);
  }
  mission_planner_ptr_->setSolvedTrajectories(path, id);
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

void MissionPlannerRos::setMarkerColor(visualization_msgs::Marker &marker,
                                       const Colors &color) {
  switch (color) {
    case Colors::RED:
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      break;
    case Colors::BLUE:
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      break;
    case Colors::YELLOW:
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      break;
    default:
      break;
  }
}

void MissionPlannerRos::publishSphere(const ros::Publisher &pub_sphere,
                                      const Colors &color) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = param_.frame;
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  // set position
  Eigen::Vector3d point_to_inspect = mission_planner_ptr_->getPointToInspect();
  marker.pose.position.x = point_to_inspect(0);
  marker.pose.position.y = point_to_inspect(1);
  marker.pose.position.z = point_to_inspect(2);
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = mission_planner_ptr_->getDistanceToInspect() * 2;
  marker.scale.y = mission_planner_ptr_->getDistanceToInspect() * 2;
  marker.scale.z = 25;
  marker.color.a = 0.4;
  // inspection distance
  setMarkerColor(marker, Colors::YELLOW);
  // only if using a MESH_RESOURCE marker type:
  marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  pub_sphere.publish(marker);
}

void MissionPlannerRos::publishPoints(
    const ros::Publisher &pub_points,
    const std::vector<geometry_msgs::Point> &_points, Colors color) {
  visualization_msgs::Marker points;

  // markers config
  points.header.frame_id = param_.frame;
  points.header.stamp = ros::Time::now();
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.x = 0.0;
  points.pose.orientation.y = 0.0;
  points.pose.orientation.z = 0.0;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.2;
  points.scale.y = 0.2;
  setMarkerColor(points, color);
  points.color.g = 1.0f;
  points.color.a = 1.0;

  points.points = _points;
  pub_points.publish(points);
}

void MissionPlannerRos::publishDistance(const ros::Publisher &pub_distance) {
  mission_planner::Float32withHeader dist;

  dist.header.frame_id = param_.frame;
  dist.header.stamp = ros::Time::now();
  dist.data = mission_planner_ptr_ -> getDistanceToInspect();

  pub_distance.publish(dist);

}

void MissionPlannerRos::publishRelativeAngle(const ros::Publisher &pub_angle) {
  mission_planner::Float32withHeader angle;

  angle.header.frame_id = param_.frame;
  angle.header.stamp = ros::Time::now();
  angle.data = mission_planner_ptr_ -> getRelativeAngle();

  pub_angle.publish(angle);

}

void MissionPlannerRos::publishMissionStatus(const ros::Publisher &pub_status) {
  mission_planner::BoolWithHeader mission_status;

  mission_status.header.frame_id = param_.frame;
  mission_status.header.stamp = ros::Time::now();
  mission_status.data = mission_planner_ptr_ -> getMissionStatus();

  pub_status.publish(mission_status);
}

void MissionPlannerRos::publishPath(const ros::Publisher &pub_path,
                                    const std::vector<state> &trajectory) {
  nav_msgs::Path path_to_publish;
  geometry_msgs::PoseStamped aux_pose;
  path_to_publish.header.frame_id = param_.frame;
  path_to_publish.header.stamp = ros::Time::now();
  for (const auto &state : trajectory) {
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
  } catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'",
              pub_path.getTopic().c_str());
  }
}

void MissionPlannerRos::publishTrajectoryJoint(
    const ros::Publisher &pub_path, const std::vector<state> &trajectory) {
  trajectory_msgs::JointTrajectory trajectory_to_follow;
  trajectory_msgs::JointTrajectoryPoint point_to_follow;
  for (auto &point : trajectory) {
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
  } catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'",
              pub_path.getTopic().c_str());
  }
}
