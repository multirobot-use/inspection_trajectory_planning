#include "mission_planner_ros.hpp" 
MissionPlannerRos::MissionPlannerRos(ros::NodeHandle _nh, const bool leader)
    : nh_(_nh) {

  // ROS params
  trajectory_planner::safeGetParam(nh_, "horizon_length", param_.horizon_length);
  trajectory_planner::safeGetParam(nh_, "n_drones", param_.n_drones);
  trajectory_planner::safeGetParam(nh_, "step_size", param_.step_size);
  trajectory_planner::safeGetParam(nh_, "planning_rate", param_.planning_rate);
  trajectory_planner::safeGetParam(nh_, "topics_rate", param_.topics_rate);
  trajectory_planner::safeGetParam(nh_, "drone_id", param_.drone_id);
  trajectory_planner::safeGetParam(nh_, "vel_max", param_.vel_max);
  trajectory_planner::safeGetParam(nh_, "vel_min", param_.vel_min);
  trajectory_planner::safeGetParam(nh_, "vel_inspect", param_.vel_inspect);
  trajectory_planner::safeGetParam(nh_, "orbit_time", param_.orbit_time);
  trajectory_planner::safeGetParam(nh_, "acc_max", param_.acc_max);
  trajectory_planner::safeGetParam(nh_, "frame", param_.frame);
  trajectory_planner::safeGetParam(nh_, "operation_mode", param_.operation_mode);
  trajectory_planner::safeGetParam(nh_, "inspection_dist", inspection_params_.inspection_dist);
  trajectory_planner::safeGetParam(nh_, "min_inspection_dist", inspection_params_.min_inspection_dist);
  trajectory_planner::safeGetParam(nh_, "max_inspection_dist", inspection_params_.max_inspection_dist);
  trajectory_planner::safeGetParam(nh_, "visualization_rate", param_.visualization_rate);
  trajectory_planner::safeGetParam(nh_, "clock_rate", param_.clock_rate);
  trajectory_planner::safeGetParam(nh_, "leader_id", inspection_params_.leader_id);
  trajectory_planner::safeGetParam(nh_, "inc_distance", inspection_params_.inc_distance);
  trajectory_planner::safeGetParam(nh_, "inc_angle", inspection_params_.inc_angle);
  trajectory_planner::safeGetParam(nh_, "inc_orbit_time", inspection_params_.inc_orbit_time);
  trajectory_planner::safeGetParam(nh_, "pcl_filepath", param_.pcd_file_path);
  trajectory_planner::safeGetParam(nh_, "static_map", param_.static_map);
  trajectory_planner::safeGetParam(nh_, "obstacle_avoidance", param_.obstacle_avoidance);
  trajectory_planner::safeGetParam(nh_, "opt_orientation", param_.opt_orientation);

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

    cur_state_sub_[drone] = nh_.subscribe<uav_abstraction_layer::State>(
        "/drone_" + std::to_string(drone) + "/ual/state", 1,
        std::bind(&MissionPlannerRos::uavStateCallback, this,
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
    
    orbit_time_sub_[drone] = nh_.subscribe<std_msgs::Bool>(
        "/drone_" + std::to_string(drone) +
            "/mission_planner_ros/orbit_time_joy",
        1,
        std::bind(&MissionPlannerRos::orbitTimeJoyCallback, this,
                  std::placeholders::_1, drone));

    if (drone != param_.drone_id) {
      solved_trajectories_sub_[drone] = nh_.subscribe<nav_msgs::Path>(
          "/drone_" + std::to_string(drone) +
              "/mission_planner_ros/solved_traj",
          1,
          std::bind(&MissionPlannerRos::solvedTrajCallback, this,
                    std::placeholders::_1, drone));
      
      reference_trajectories_sub_[drone] = nh_.subscribe<nav_msgs::Path>(
          "/drone_" + std::to_string(drone) +
              "/mission_planner_ros/reference_traj",
          1,
          std::bind(&MissionPlannerRos::referenceTrajCallback, this,
                    std::placeholders::_1, drone));
    }
    if (drone != inspection_params_.leader_id) {
      operation_mode_sub_[drone] = nh_.subscribe<std_msgs::UInt8>(
          "/drone_1/operation_mode",
          1,
          std::bind(&MissionPlannerRos::operationModeCallback, this,
                    std::placeholders::_1, drone));
      planner_status_sub_[drone] = nh_.subscribe<std_msgs::UInt8>(
          "/drone_1/planner_status",
          1,
          std::bind(&MissionPlannerRos::plannerStatusCallback, this,
                    std::placeholders::_1, drone));
      waypoints_sub_[drone] = nh_.subscribe<geometry_msgs::PoseArray>(
          "/drone_1/mission_planner_ros/waypoints",
          1,
          std::bind(&MissionPlannerRos::waypointsCallback, this,
                    std::placeholders::_1, drone));
    }
  }

  // LiDAR
  if (param_.static_map =! 1){
    if (param_.drone_id == 1){
      ops[1] =
          ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
              "/drone_1/os1_cloud_node/points",  // topic name
              1,                             // queue length
              std::bind(&MissionPlannerRos::pcdCallback, this,
                          std::placeholders::_1, 1),
              ros::VoidPtr(),
              &this->pcd_queue_1_  // pointer to callback queue object
          );
      ops[1].transport_hints = ros::TransportHints().tcpNoDelay();
      pcd_sub_[1] = nh_.subscribe(ops[1]);
      async_spinner_1_.start();
    }
    else if (param_.drone_id == 2){
      ops[2] =
          ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
              "/drone_2/os1_cloud_node/points",  // topic name
              1,                             // queue length
              std::bind(&MissionPlannerRos::pcdCallback, this,
                          std::placeholders::_1, 2),
              ros::VoidPtr(),
              &this->pcd_queue_2_  // pointer to callback queue object
          );

      ops[2].transport_hints = ros::TransportHints().tcpNoDelay();
      pcd_sub_[2] = nh_.subscribe(ops[2]);
      async_spinner_2_.start();
    }
    else{
      ops[3] =
          ros::SubscribeOptions::create<sensor_msgs::PointCloud2>(
              "/drone_3/os1_cloud_node/points",  // topic name
              1,                             // queue length
              std::bind(&MissionPlannerRos::pcdCallback, this,
                          std::placeholders::_1, 3),
              ros::VoidPtr(),
              &this->pcd_queue_3_  // pointer to callback queue object
          );
        
      ops[3].transport_hints = ros::TransportHints().tcpNoDelay();
      pcd_sub_[3] = nh_.subscribe(ops[3]);
      async_spinner_3_.start();

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
      nh_.advertise<visualization_msgs::Marker>("waypoints_to_inspect", 1);
  points_trans_pub_ = nh_.advertise<visualization_msgs::Marker>(
      "waypoints_to_inspect_transformed", 1);
  waypoints_pub_ =
      nh_.advertise<geometry_msgs::PoseArray>("waypoints", 1);
  point_to_inspect_pub_ = 
      nh_.advertise<geometry_msgs::Pose>("point_to_inspect", 1);
  sphere_pub_ =
      nh_.advertise<visualization_msgs::Marker>("inspection_sphere", 1);
  pub_path_ = nh_.advertise<nav_msgs::Path>("solved_traj", 1);
  pub_ref_path_ = nh_.advertise<nav_msgs::Path>("reference_traj", 1);
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
  orbit_time_pub_ = nh_.advertise<mission_planner::Float32withHeader>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/orbit_time",
      10);
  mission_status_pub_ = nh_.advertise<mission_planner::BoolWithHeader>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/mission_status",
      1);
  operation_mode_pub_ = nh_.advertise<std_msgs::UInt8>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/operation_mode",
      1);
  planner_status_pub_ = nh_.advertise<std_msgs::UInt8>(
      "/drone_" + std::to_string(param_.drone_id) +
          "/planner_status",
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
  service_waypoint_by_angle = nh_.advertiseService(
      "add_waypoint_by_angle", &MissionPlannerRos::addWaypointByAngleServiceCallback, this);
  service_point_to_inspect = nh_.advertiseService(
      "point_to_inspect", &MissionPlannerRos::pointToInspectServiceCallback,
      this);
  clear_waypoints = nh_.advertiseService(
      "clear_waypoints", &MissionPlannerRos::clearWaypointsServiceCallback,
      this);
  clear_first_waypoint = nh_.advertiseService(
      "clear_first_waypoint", &MissionPlannerRos::clearFirstWaypointServiceCallback,
      this);
  service_distance_to_inspect = nh_.advertiseService(
      "distance_to_inspect",
      &MissionPlannerRos::distanceToInspectServiceCallback, this);
  service_relative_angle = nh_.advertiseService(
      "change_relative_angle",
      &MissionPlannerRos::changeRelativeAngleServiceCallback, this);
  service_orbit_time = nh_.advertiseService(
      "orbit_time",
      &MissionPlannerRos::orbitTimeServiceCallback, this);
  service_operation_mode = nh_.advertiseService(
      "change_operation_mode",
      &MissionPlannerRos::changeOperationModeServiceCallback, this);
}


MissionPlannerRos::~MissionPlannerRos() {}


// Callbacks
bool MissionPlannerRos::activationPlannerServiceCallback(
    std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  ROS_INFO("[%s]: Activation planner service called.",
           ros::this_node::getName().c_str());

  res.success = true;

  if (mission_planner_ptr_-> getDroneStatus(param_.drone_id) == 4){
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
  }
  else{
    ROS_INFO("[%s]: Activate Planner unavailable, drone is not flying auto. Please, arm and take off the drone", ros::this_node::getName().c_str());
    res.message = "Activate Planner unavailable, drone is not flying auto. Please, arm and take off the drone.";
    res.success = false;
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
  res.message = "Waypoint added successfully!";

  return true;
}

bool MissionPlannerRos::addWaypointByAngleServiceCallback(
    mission_planner::AddWaypointByAngle::Request &req,
    mission_planner::AddWaypointByAngle::Response &res){
  ROS_INFO("[%s]: Add waypoint by angle service called.",
           ros::this_node::getName().c_str());

  geometry_msgs::Point point;

  point.x = 1*cos(req.angle);
  point.y = 1*sin(req.angle);
  point.z = req.height;
  points_.push_back(std::move(point));

  state state_req;

  state_req.pos(0) = point.x;
  state_req.pos(1) = point.y;
  state_req.pos(2) = point.z;

  mission_planner_ptr_->appendGoal(state_req);

  res.success = true;
  res.message = "Waypoint added successfully!";

  return true;
}

bool MissionPlannerRos::clearWaypointsServiceCallback(
    std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("[%s]: Clear waypoints service called.",
           ros::this_node::getName().c_str());

  mission_planner_ptr_->clearGoals();

  return true;
}

bool MissionPlannerRos::clearFirstWaypointServiceCallback(
    std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  ROS_INFO("[%s]: Clear first waypoint service called.",
           ros::this_node::getName().c_str());

  mission_planner_ptr_->clearFirstGoal();

  return true;
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
  res.message = "Waypoint to inspect changed successfully!";

  return true;
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
  res.message = "Inspection distance changed successfully!";

  return true;
}

bool MissionPlannerRos::orbitTimeServiceCallback(
    mission_planner::OrbitTimeSrv::Request &req,
    mission_planner::OrbitTimeSrv::Response &res) {
  ROS_INFO("[%s]: Orbit time service called.",
           ros::this_node::getName().c_str());

  mission_planner_ptr_->setOrbitTime(req.time);

  ROS_INFO(
      "[%s]: Orbit time changed successfully!  New time: "
      " %f seconds",
      ros::this_node::getName().c_str(), req.time);
  res.success = true;
  res.message = "Orbit time changed successfully!";

  return true;
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
  res.message = "Formation angle added successfully!";

  return true;
}

bool MissionPlannerRos::changeOperationModeServiceCallback(
    mission_planner::OperationModeSrv::Request &req,
    mission_planner::OperationModeSrv::Response &res) {

  ROS_INFO("[%s]: Change operation mode service called.",
           ros::this_node::getName().c_str());
  
  mission_planner_ptr_ -> setOperationMode(req.mode);

  ROS_INFO("[%s]: Operation mode changed successfully!  New mode: %d",
           ros::this_node::getName().c_str(), req.mode);
  res.success = true;
  res.message = "Operation mode changed successfully!";

  return true;
}

void MissionPlannerRos::pcdCallback(
    const sensor_msgs::PointCloud2::ConstPtr &msg, const int id) {

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_received(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_transformed(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::fromROSMsg(*msg, *pcl_cloud_received);

  geometry_msgs::TransformStamped Tstatic_frame_velodyne_frame;

  // bool can_transform;
  // can_transform = tfBuffer.canTransform("map", "drone_" + std::to_string(param_.drone_id) + "/velodyne",
        // ros::Time(0));
  // can_transform = tfBuffer._frameExists("drone_" + std::to_string(param_.drone_id) + "/velodyne");
  
  // ROS_INFO("Can transform %d", can_transform);

  try {
    Tstatic_frame_velodyne_frame = tfBuffer[id].lookupTransform(
        "map", "drone_" + std::to_string(id) + "/base_link",
        ros::Time(0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return;
  }
  Eigen::Affine3d transformation =
      tf2::transformToEigen(Tstatic_frame_velodyne_frame);

  pcl::transformPointCloud(*pcl_cloud_received, *pcl_cloud_transformed,
                           transformation);

  mission_planner_ptr_ -> updateMap(pcl_cloud_transformed);
}

void MissionPlannerRos::replanCB(const ros::TimerEvent &e) {

  // Refresh the Waypoint Tolerance value
  float distance = mission_planner_ptr_ -> getDistanceToInspect();
  float vel      = mission_planner_ptr_ -> calculateVel(distance);
  mission_planner_ptr_ -> adaptiveWaypointTolerance(vel);

  auto trajectory_length = mission_planner_ptr_ -> getSizeTrajectory();

  if (mission_planner_ptr_->getStatus() != trajectory_planner::PlannerStatus::FIRST_PLAN) {
    if (trajectory_length > 0) {
      publishTrajectoryJoint(tracking_pub_trajectory_,
                             mission_planner_ptr_->getLastTrajectory());
      publishPath(pub_path_, mission_planner_ptr_->getLastTrajectory());
      publishPath(pub_ref_path_, mission_planner_ptr_->getReferenceTrajectory());
      mission_planner_ptr_->safe_corridor_generator_->publishCorridor(
          corridor_pub_);
      mission_planner_ptr_->safe_corridor_generator_->updateMaps();
    }
  }
  mission_planner_ptr_->safe_corridor_generator_->publishCloud(
        pub_point_cloud_);
  mission_planner_ptr_->plan();
  publishOperationMode(operation_mode_pub_);
  publishPlannerStatus(planner_status_pub_);
  publishDistance(distance_pub_);
  publishRelativeAngle(angle_pub_);
  publishOrbitTime(orbit_time_pub_);
  publishMissionStatus(mission_status_pub_);
  publishPointToInspect(point_to_inspect_pub_);

  std::vector<state> goals = mission_planner_ptr_->getGoals();

  geometry_msgs::Pose point;
  geometry_msgs::PoseArray points;

  for (auto const &goal : goals) {
    point.position.x = goal.pos(0);
    point.position.y = goal.pos(1);
    point.position.z = goal.pos(2);
    points.poses.push_back(point);
  }
  publishWaypoints(waypoints_pub_, points);

}

void MissionPlannerRos::clockCB(const ros::TimerEvent &e) {
  auto current_time = ros::Time::now();
  float aux_time = current_time.sec + current_time.nsec/1000000000.0;
  mission_planner_ptr_->setCurrentTime(aux_time);
}

void MissionPlannerRos::topicsCB(const ros::TimerEvent &e) {
  // Refresh the maximums and minimums for the formation angle
  mission_planner_ptr_->refreshRelativeAngle();
  
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
  publishCylinder(sphere_pub_, Colors::YELLOW);
}

void MissionPlannerRos::distanceToInspectionPointCallback(
    const std_msgs::Bool::ConstPtr &distance, int id) {

  mission_planner_ptr_ -> incDistanceToInspect(distance->data);
}

void MissionPlannerRos::orbitTimeJoyCallback(
    const std_msgs::Bool::ConstPtr &time, int id) {

  mission_planner_ptr_ -> incOrbitTime(time->data);
}

void MissionPlannerRos::operationModeCallback(
    const std_msgs::UInt8::ConstPtr &operation_mode, int id){

  mission_planner_ptr_ -> setOperationMode(operation_mode->data);
}

void MissionPlannerRos::plannerStatusCallback(
    const std_msgs::UInt8::ConstPtr &planner_status, int id){

  mission_planner_ptr_ -> setStatus(planner_status->data);
}

void MissionPlannerRos::waypointsCallback(
    const geometry_msgs::PoseArray::ConstPtr &waypoints, int id){
  // Adapt to trajectory_planner::state
  std::vector<trajectory_planner::state> goals;
  trajectory_planner::state waypoint;
  goals.clear();

  for (auto i = 0; i < waypoints->poses.size(); i++){
    waypoint.pos(0) = waypoints->poses[i].position.x;
    waypoint.pos(1) = waypoints->poses[i].position.y;
    waypoint.pos(2) = waypoints->poses[i].position.z;
    goals.push_back(waypoint);
  }  

  mission_planner_ptr_ -> setGoals(goals);
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

  for (auto pose : msg->poses) {
    aux_state.time_stamp = pose.header.stamp.sec + pose.header.stamp.nsec/1000000000.0; 

    i = i + 1;
    aux_state.pos(0) = pose.pose.position.x;
    aux_state.pos(1) = pose.pose.position.y;
    aux_state.pos(2) = pose.pose.position.z;
    path.push_back(aux_state);
  }
  mission_planner_ptr_->setSolvedTrajectories(path, id);
}

void MissionPlannerRos::referenceTrajCallback(const nav_msgs::Path::ConstPtr &msg,
                                              int id) {
  state aux_state;
  std::vector<state> path;

  auto time_first_point = ros::Time::now();
  float time = time_first_point.sec + (time_first_point.nsec / 1000000000.0);
  int i = 0;

  for (auto pose : msg->poses) {
    aux_state.time_stamp = pose.header.stamp.sec + pose.header.stamp.nsec/1000000000.0; 

    i = i + 1;
    aux_state.pos(0) = pose.pose.position.x;
    aux_state.pos(1) = pose.pose.position.y;
    aux_state.pos(2) = pose.pose.position.z;
    path.push_back(aux_state);
  }
  mission_planner_ptr_->setReferenceTrajectories(path, id);
}

void MissionPlannerRos::uavPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg, int id) {
  mission_planner_ptr_->states_[id].pos[0] = msg->pose.position.x;
  mission_planner_ptr_->states_[id].pos[1] = msg->pose.position.y;
  mission_planner_ptr_->states_[id].pos[2] = msg->pose.position.z;
}

void MissionPlannerRos::uavStateCallback(
    const uav_abstraction_layer::State::ConstPtr &msg, int id) {
  mission_planner_ptr_->setDroneStatus(param_.drone_id, msg->state);
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

void MissionPlannerRos::publishCylinder(const ros::Publisher &pub_cylinder,
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
  pub_cylinder.publish(marker);
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

void MissionPlannerRos::publishWaypoints(
  const ros::Publisher &pub_waypoints,
  const geometry_msgs::PoseArray &_points){

  geometry_msgs::PoseArray points;

  points.header.frame_id = param_.frame;
  points.header.stamp = ros::Time::now();
  points.poses = _points.poses;

  pub_waypoints.publish(points);
}

void MissionPlannerRos::publishOperationMode(const ros::Publisher &pub_operation_mode){
  std_msgs::UInt8 operation_mode;

  operation_mode.data = mission_planner_ptr_ -> getOperationMode();

  pub_operation_mode.publish(operation_mode);
}

void MissionPlannerRos::publishPlannerStatus(const ros::Publisher &pub_planner_status){
  std_msgs::UInt8 planner_status;

  planner_status.data = mission_planner_ptr_ -> getStatus();

  pub_planner_status.publish(planner_status);
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

void MissionPlannerRos::publishOrbitTime(const ros::Publisher &pub_orbit_time) {
  mission_planner::Float32withHeader orbit_time;

  orbit_time.header.frame_id = param_.frame;
  orbit_time.header.stamp    = ros::Time::now();
  orbit_time.data = mission_planner_ptr_ -> getOrbitTime();

  pub_orbit_time.publish(orbit_time);
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

void MissionPlannerRos::publishPointToInspect(const ros::Publisher &pub_point_to_inspect) {
  geometry_msgs::Pose point_to_inspect;
  Eigen::Vector3d     point_to_inspect_v = mission_planner_ptr_ -> getPointToInspect();

  point_to_inspect.position.x = point_to_inspect_v[0];
  point_to_inspect.position.y = point_to_inspect_v[1];
  point_to_inspect.position.z = point_to_inspect_v[2];

  pub_point_to_inspect.publish(point_to_inspect);
}

void MissionPlannerRos::publishPath(const ros::Publisher &pub_path,
                                    const std::vector<state> &trajectory) {
  if (!trajectory.empty()){
    nav_msgs::Path path_to_publish;
    geometry_msgs::PoseStamped aux_pose;

    path_to_publish.header.frame_id = param_.frame;
    path_to_publish.header.stamp    = ros::Time::now();

    int i = 0;
    float time_stamp = trajectory[0].time_stamp;
    div_t time_result;

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
  
}

void MissionPlannerRos::publishTrajectoryJoint(
    const ros::Publisher &pub_path, const std::vector<state> &trajectory) {
  
  if (!trajectory.empty()){
    trajectory_msgs::JointTrajectory trajectory_to_follow;
    trajectory_msgs::JointTrajectoryPoint point_to_follow;

    ros::Time aux;
    ros::Time zero(0, 0);

    for (auto &point : trajectory) {
      point_to_follow.positions.clear();
      point_to_follow.velocities.clear();
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
}
