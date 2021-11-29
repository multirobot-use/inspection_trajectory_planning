#include "mission_planner_ros.hpp" 
MissionPlannerRos::MissionPlannerRos(ros::NodeHandle _nh, const bool leader)
    : nh_(_nh) {

  // ROS params
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
  trajectory_planner::safeGetParam(nh_, "visualization_rate", param_.visualization_rate);
  trajectory_planner::safeGetParam(nh_, "clock_rate", param_.clock_rate);
  trajectory_planner::safeGetParam(nh_, "leader_id", inspection_params_.leader_id);
  trajectory_planner::safeGetParam(nh_, "inc_distance", inspection_params_.inc_distance);
  trajectory_planner::safeGetParam(nh_, "inc_angle", inspection_params_.inc_angle);
  trajectory_planner::safeGetParam(nh_,"pcl_filepath", param_.pcd_file_path);

  // Initialize mission planner
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
        20,
        std::bind(&MissionPlannerRos::distanceToInspectionPointCallback, this,
                  std::placeholders::_1, drone));

    relative_angle_sub_[drone] = nh_.subscribe<std_msgs::Bool>(
        "/drone_" + std::to_string(drone) +
            "/mission_planner_ros/relative_angle",
        20,
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

  // Create timers
  planTimer_ = nh_.createTimer(ros::Duration(param_.planning_rate),
                               &MissionPlannerRos::replanCB, this);
  clockTimer_ = nh_.createTimer(ros::Duration(param_.clock_rate),
                            &MissionPlannerRos::clockCB, this);
  pubVis_ = nh_.createTimer(ros::Duration(param_.visualization_rate),
                            &MissionPlannerRos::pubVisCB, this);
  topicsTimer_ = nh_.createTimer(ros::Duration(param_.topics_rate),
                            &MissionPlannerRos::topicsCB, this);
  clockTimer_.start();
  planTimer_.stop();
  pubVis_.start();

  // Publishers
  corridor_pub_ =
      nh_.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedrons_out", 1);
  pub_point_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("pcl_map_out", 1);
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
      10);
  angle_pub_ = nh_.advertise<mission_planner::Float32withHeader>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/absolute_relative_angle",
      10);
  mission_status_pub_ = nh_.advertise<mission_planner::BoolWithHeader>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/mission_status",
      1);
  if (param_.drone_id != inspection_params_.leader_id){
    formation_angle_pub_ = nh_.advertise<mission_planner::Float32withHeader>(
        "/drone_" + std::to_string(param_.drone_id) +
            "/formation_angle",
        1);
  }

  inspection_distance_pub_ = nh_.advertise<mission_planner::Float32withHeader>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/inspection_distance",
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
    publishPath(pub_path_, mission_planner_ptr_->getLastTrajectory());
    publishPath(pub_ref_path_, mission_planner_ptr_->getReferenceTrajectory());
    mission_planner_ptr_->safe_corridor_generator_->publishCorridor(
        corridor_pub_);
  }
  mission_planner_ptr_->plan();
  publishDistance(distance_pub_);
  publishRelativeAngle(angle_pub_);
  publishMissionStatus(mission_status_pub_);
}

void MissionPlannerRos::clockCB(const ros::TimerEvent &e) {
  auto current_time = ros::Time::now();
  float aux_time = current_time.sec + current_time.nsec/1000000000.0;
  mission_planner_ptr_->setCurrentTime(aux_time);

  
}

void MissionPlannerRos::topicsCB(const ros::TimerEvent &e) {
  if (param_.drone_id != inspection_params_.leader_id)  publishFormationAngle(formation_angle_pub_);

  publishInspectionDistance(inspection_distance_pub_);

}

void MissionPlannerRos::pubVisCB(const ros::TimerEvent &e) {
  // Publish commanded waypoint
  publishPoints(points_pub_, points_, Colors::RED);

  // Publish transformed waypoints
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

  auto time_first_point = ros::Time::now();
  float time = time_first_point.sec + (time_first_point.nsec / 1000000000.0);
  int i = 0;

  // ROS_INFO("SOLVED TRAJ CALLBACK: Time of solved trajectory ID %d callback:  %3f seconds", id, time);
  for (auto pose : msg->poses) {
    // Check how would improve the formation angle by using the time_first_point instead of the poses one.
    aux_state.time_stamp = pose.header.stamp.sec + pose.header.stamp.nsec/1000000000.0; 

    i = i + 1;
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

  // Set position
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

  // Inspection distance
  setMarkerColor(marker, Colors::YELLOW);

  // Only if using a MESH_RESOURCE marker type:
  marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  pub_sphere.publish(marker);
}

void MissionPlannerRos::publishPoints(
    const ros::Publisher &pub_points,
    const std::vector<geometry_msgs::Point> &_points, Colors color) {
  visualization_msgs::Marker points;

  // Markers config
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
  dist.header.stamp    = ros::Time::now();
  dist.data = mission_planner_ptr_ -> getDistanceToInspect();

  pub_distance.publish(dist);
}

void MissionPlannerRos::publishInspectionDistance(const ros::Publisher &pub_distance){
  mission_planner::Float32withHeader dist;

  dist.header.frame_id = param_.frame;
  dist.header.stamp    = ros::Time::now();
  dist.data = mission_planner_ptr_ -> getInspectionDistance(param_.drone_id);

  pub_distance.publish(dist);
}

void MissionPlannerRos::publishRelativeAngle(const ros::Publisher &pub_angle) {
  mission_planner::Float32withHeader angle;

  angle.header.frame_id = param_.frame;
  angle.header.stamp    = ros::Time::now();
  angle.data = mission_planner_ptr_ -> getRelativeAngle();

  pub_angle.publish(angle);
}

void MissionPlannerRos::publishFormationAngle(const ros::Publisher &pub_angle) {
  mission_planner::Float32withHeader angle;

  angle.header.frame_id = param_.frame;
  angle.header.stamp    = ros::Time::now();
  // angle.data = mission_planner_ptr_ -> calculateFormationAngle(param_.drone_id);
  angle.data = mission_planner_ptr_ -> getFormationAngle(param_.drone_id);

  pub_angle.publish(angle);
}

void MissionPlannerRos::publishMissionStatus(const ros::Publisher &pub_status) {
  mission_planner::BoolWithHeader mission_status;

  mission_status.header.frame_id = param_.frame;
  mission_status.header.stamp    = ros::Time::now();
  mission_status.data = mission_planner_ptr_ -> getMissionStatus();

  pub_status.publish(mission_status);
}

void MissionPlannerRos::publishPath(const ros::Publisher &pub_path,
                                    const std::vector<state> &trajectory) {
  nav_msgs::Path path_to_publish;
  geometry_msgs::PoseStamped aux_pose;

  path_to_publish.header.frame_id = param_.frame;
  path_to_publish.header.stamp    = ros::Time::now();

  int i = 0;
  float time_stamp = trajectory[0].time_stamp;
  div_t time_result;

  // ROS_INFO("PUBLISH PATH METHOD: Current time sec: %3d  nsec: %3d", path_to_publish.header.stamp.sec, path_to_publish.header.stamp.nsec);
  // ROS_INFO("PUBLISH PATH METHOD: Time of the first point of the trajectory: %f", time_stamp);

  for (const auto &state : trajectory) {
    time_result = div(state.time_stamp, 1);
    aux_pose.header.stamp.sec   = time_result.quot;
    aux_pose.header.stamp.nsec  = int((state.time_stamp - time_result.quot)*1000000000); // time_result.rem*1000000000 not working fine

    aux_pose.pose.position.x = state.pos(0);
    aux_pose.pose.position.y = state.pos(1);
    aux_pose.pose.position.z = state.pos(2);

    aux_pose.pose.orientation.x = state.orientation.x();
    aux_pose.pose.orientation.y = state.orientation.y();
    aux_pose.pose.orientation.z = state.orientation.z();
    aux_pose.pose.orientation.w = state.orientation.w();

    path_to_publish.poses.push_back(aux_pose);
    i = i + 1;
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

  ros::Time aux;
  ros::Time zero(0, 0);

  for (auto &point : trajectory) {
    point_to_follow.positions.clear();
    point_to_follow.velocities.clear();
    // std::cout << "   Time: " << point.time_stamp << std::endl;
    aux.sec  = int(point.time_stamp);
    aux.nsec = int( (point.time_stamp-int(point.time_stamp))*1000000000 );
    point_to_follow.time_from_start = aux - zero;
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
