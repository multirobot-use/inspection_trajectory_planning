#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <handy_tools/pid_controller.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <Eigen/Eigen>
#include <chrono>
#include <thread>

const float YAW_PID_P{0.4};
const float YAW_PID_I{0.02};
const float YAW_PI_D{0.0};

struct Trajectory {
  std::vector<Eigen::Vector3f> positions;   // trajectory to follow
  std::vector<Eigen::Vector3f> velocities;  // trajectory to follow
  std::vector<Eigen::Quaternionf> orientations;
  std::vector<float> times;

  Trajectory(const int N, const float step_size) {
    std::wcout << "Fixed horizon and step size";
    for (int i = 0; i < N; i++) {
      times.push_back(i * step_size);
    }
  }
  Trajectory() {}

  bool calculateTimes() {
    if (positions.empty() || velocities.empty()) {
      return false;
    }
    times.push_back(0.0);
    for (int i = 0; i < positions.size() - 1; i++) {
      times.push_back((positions[i + 1] - positions[i]).norm() /
                      velocities[i].norm());
    }
  }
};
struct State {
  Eigen::Vector3f current_pose;
  Eigen::Quaternionf current_orientation;
  Eigen::Vector3f current_vel;
};

struct Follower {
  const double look_ahead{1.0};
  int pose_on_path{0};
  const float rate{0.01};  // hz
  const float velocity_error{0.1};

  std::chrono::time_point<std::chrono::high_resolution_clock> time_last_traj;

  float calculateYawDiff(const float _desired_yaw, const float _current_yaw);
  int cal_pose_on_path(const std::vector<Eigen::Vector3f> &positions,
                       const Eigen::Vector3f &current_pose);
  int cal_pose_look_ahead(const std::vector<Eigen::Vector3f> &positions);
  Eigen::Vector3f calculate_vel(const Eigen::Vector3f &target_pose,
                                const Eigen::Vector3f &current_pose,
                                const float target_time);
};

int main(int _argc, char **_argv) {
  ros::init(_argc, _argv, "trajectory_follower_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  Trajectory last_traj_received{40, 0.1};
  State uav_state;
  Follower follower;
  //// ROS publishers and subscribers /////////////
  ros::Publisher tracking_pub =
      nh.advertise<nav_msgs::Path>("follower/trajectory_to_follow", 1);

  // trajectory callback
  auto trajectoryCallback =
      [&last_traj_received, &follower,
       tracking_pub](const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
        follower.time_last_traj = std::chrono::high_resolution_clock::now();
        follower.pose_on_path = 0;
        last_traj_received.positions.clear();
        last_traj_received.velocities.clear();
        last_traj_received.orientations.clear();

        Eigen::Vector3f velocity;
        Eigen::Vector3f position;
        Eigen::Quaternionf orientation;

        geometry_msgs::PoseStamped pose_stamped;
        nav_msgs::Path path_to_publish;
        path_to_publish.header.frame_id = "map";

        for (auto &point : msg->points) {
          pose_stamped.pose.position.x = point.positions[0];
          pose_stamped.pose.position.y = point.positions[1];
          pose_stamped.pose.position.z = point.positions[2];

          position[0] = point.positions[0];
          position[1] = point.positions[1];
          position[2] = point.positions[2];

          orientation.x() = point.positions[3];
          orientation.y() = point.positions[4];
          orientation.z() = point.positions[5];
          orientation.w() = point.positions[6];

          velocity[0] = point.velocities[0];
          velocity[1] = point.velocities[1];
          velocity[2] = point.velocities[2];

          last_traj_received.velocities.push_back(velocity);
          last_traj_received.positions.push_back(position);
          last_traj_received.orientations.push_back(orientation);
          path_to_publish.poses.push_back(pose_stamped);
        }
        tracking_pub.publish(path_to_publish);
      };
  auto sub_traj = pnh.subscribe<trajectory_msgs::JointTrajectory>(
      "trajectory_to_follow", 1, trajectoryCallback);
  // ual state subscriber
  auto ualPoseCallback = [&uav_state](
                             const geometry_msgs::PoseStamped::ConstPtr &msg) {
    uav_state.current_pose = Eigen::Vector3f(
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    uav_state.current_orientation =
        Eigen::Quaternionf(msg->pose.orientation.x, msg->pose.orientation.y,
                           msg->pose.orientation.z, msg->pose.orientation.w);
  };
  auto pose_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("ual/pose", 1, ualPoseCallback);

  ros::Publisher velocity_ual_pub =
      nh.advertise<geometry_msgs::TwistStamped>("ual/set_velocity", 1);
  /////////////////////////
  // PID controller for yaw 
  grvc::utils::PidController yaw_pid("yaw", YAW_PID_P, YAW_PID_I, YAW_PID_P);

  /////// main loop   //////////////
  while (ros::ok) {
    // wait for receiving trajectories
    while (!last_traj_received.positions.empty()) {
      follower.pose_on_path = follower.cal_pose_on_path(
          last_traj_received.positions, uav_state.current_pose);
      int target_pose_idx =
          follower.cal_pose_look_ahead(last_traj_received.positions);
      // if the point to go is out of the trajectory, the trajectory will be
      // finished and cleared
      if (target_pose_idx == last_traj_received.positions.size()) {
        last_traj_received.positions.clear();
        last_traj_received.velocities.clear();
        follower.pose_on_path = 0;
        break;
      }
      Eigen::Vector3f target_pose =
          last_traj_received.positions[target_pose_idx];

      Eigen::Vector3f velocity_to_command =
          follower.calculate_vel(target_pose, uav_state.current_pose,
                                 last_traj_received.times[target_pose_idx]);

      // yaw
      tf2::Quaternion desired_q(
          last_traj_received.orientations[target_pose_idx].x(),
          last_traj_received.orientations[target_pose_idx].y(),
          last_traj_received.orientations[target_pose_idx].z(),
          last_traj_received.orientations[target_pose_idx].w());
      tf2::Quaternion current_q(
          uav_state.current_orientation.x(), uav_state.current_orientation.y(),
          uav_state.current_orientation.z(), uav_state.current_orientation.w());

      float current_yaw = tf2::getYaw(current_q);
      float desired_yaw = tf2::getYaw(desired_q);
      float yaw_diff = follower.calculateYawDiff(desired_yaw, current_yaw);
      float sampling_period = follower.rate;

      // publish topic to ual
      geometry_msgs::TwistStamped vel;
      vel.header.frame_id = "map";
      vel.twist.linear.x = velocity_to_command.x();
      vel.twist.linear.y = velocity_to_command.y();
      vel.twist.linear.z = velocity_to_command.z();
      vel.twist.angular.z = yaw_pid.control_signal(yaw_diff, sampling_period);

      velocity_ual_pub.publish(vel);
      ros::spinOnce();
      ros::Duration(follower.rate).sleep();
    }
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
  return 0;
}

float Follower::calculateYawDiff(const float _desired_yaw,
                                 const float _current_yaw) {
  float yaw_diff = _desired_yaw - _current_yaw;
  while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
  while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
  return yaw_diff;
}

int Follower::cal_pose_on_path(const std::vector<Eigen::Vector3f> &positions,
                               const Eigen::Vector3f &current_pose) {
  double min_distance = INFINITY;
  int pose_on_path_id = 0;
  for (int i = pose_on_path; i < positions.size(); i++) {
    if ((current_pose - positions[i]).norm() < min_distance) {
      min_distance = (current_pose - positions[i]).norm();
      pose_on_path_id = i;
    }
  }
  return pose_on_path_id;
}

int Follower::cal_pose_look_ahead(
    const std::vector<Eigen::Vector3f> &positions) {
  for (int i = pose_on_path; i < positions.size(); i++) {
    if ((positions[i] - positions[pose_on_path]).norm() > look_ahead) return i;
  }
  return positions.size();
}

Eigen::Vector3f Follower::calculate_vel(const Eigen::Vector3f &target_pose,
                                        const Eigen::Vector3f &current_pose,
                                        const float target_time) {
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<float> current_time = end - time_last_traj;

  Eigen::Vector3f vel_unitary = (target_pose - current_pose).normalized();
  double vel_module = (target_pose - current_pose).norm() /
                      (target_time - current_time.count());
  return vel_unitary * vel_module;
}