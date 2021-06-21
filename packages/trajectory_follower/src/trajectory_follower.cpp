#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TwistStamped.h>
#include <handy_tools/pid_controller.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <tf2/utils.h>  // to convert quaternion to roll-pitch-yaw
#include <trajectory_msgs/JointTrajectory.h>
#include <uav_abstraction_layer/GoToWaypoint.h>
#include <uav_abstraction_layer/TakeOff.h>
#include <Eigen/Eigen>
#include <fstream>
#include <iostream>

const float YAW_PID_P = 0.4;
const float YAW_PID_I = 0.02;
const float YAW_PI_D = 0.0;

struct Trajectory {
  std::vector<geometry_msgs::PoseStamped> positions;  // trajectory to follow
  std::vector<geometry_msgs::Twist> velocities;       // trajectory to follow
};
struct State {
  Eigen::Vector3f current_pose;
  geometry_msgs::Quaternion current_orientation;
  Eigen::Vector3f current_vel;
};

struct Follower {
  const double look_ahead = 1.0;
};

int pose_on_path = 0;

int target_pose;  // look ahead pose
int drone_id = 1;
const float velocity_error = 0.1;
const float rate = 0.01;  // seg

float calculateYawDiff(float _desired_yaw, float _current_yaw) {
  float yaw_diff = _desired_yaw - _current_yaw;
  while (yaw_diff < -M_PI) yaw_diff += 2 * M_PI;
  while (yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
  return yaw_diff;
}

/** \brief Utility function to calculate the nearest pose on the path
 *  \param positions a path to follow
 *  \return index of the nearest pose on the path
 */
int cal_pose_on_path(const std::vector<geometry_msgs::PoseStamped> &positions,
                     const Eigen::Vector3f &current_pose,
                     int previous_pose_on_path) {
  double min_distance = 10000000;
  int pose_on_path_id = 0;
  for (int i = previous_pose_on_path; i < positions.size(); i++) {
    Eigen::Vector3f pose_on_path = Eigen::Vector3f(
        positions[i].pose.position.x, positions[i].pose.position.y,
        positions[i].pose.position.z);
    if ((current_pose - pose_on_path).norm() < min_distance) {
      min_distance = (current_pose - pose_on_path).norm();
      pose_on_path_id = i;
    }
  }
  return pose_on_path_id;
}

/** \brief Utility function to calculate the look ahead position
 *  \param positions a path to follow
 *  \param look_ahead
 *  \pose_on_path pose from which we apply look ahead
 *  \return look ahead position index
 */

int cal_pose_look_ahead(
    const std::vector<geometry_msgs::PoseStamped> &positions,
    const double look_ahead, int pose_on_path) {
  for (int i = pose_on_path; i < positions.size(); i++) {
    Eigen::Vector3f aux = Eigen::Vector3f(
        positions[i].pose.position.x - positions[pose_on_path].pose.position.x,
        positions[i].pose.position.y - positions[pose_on_path].pose.position.y,
        positions[i].pose.position.z - positions[pose_on_path].pose.position.z);
    double distance = aux.norm();
    if (distance > look_ahead) return i;
  }
  return positions.size();
}

/** \brief utility function to calculate velocity commands. This function apply
 * the direction to the next point of the trajectory and the velocity of the
 * nearest point of the trajectory. \param pose desired position of the path
 *  \param vel desired velocity of the nearest pose on the path
 *  \return 3d vector velocity to command
 */
Eigen::Vector3f calculate_vel(const Eigen::Vector3f pose,
                              const Eigen::Vector3f vel,
                              const Eigen::Vector3f &current_pose) {
  Eigen::Vector3f vel_to_command = (pose - current_pose).normalized();
  double vel_module = vel.norm() + velocity_error;
  /**if(vel_module<0.15){
      vel_module = 0.15;
  }
   std::cout<<vel_module<<std::endl;*/
  return vel_to_command * vel_module;
}

int main(int _argc, char **_argv) {
  ros::init(_argc, _argv, "trajectory_follower_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  Trajectory last_traj_received;
  State uav_state;
  Follower follower;

  ros::Publisher tracking_pub =
      nh.advertise<nav_msgs::Path>("follower/trajectory_to_follow", 1);

  // trajectory subscriber
  auto trajectoryCallback =
      [&last_traj_received,
       tracking_pub](const trajectory_msgs::JointTrajectory::ConstPtr &msg) {
        ROS_INFO("Drone %d: trajectory received", drone_id);
        pose_on_path = 0;
        last_traj_received.positions.clear();
        last_traj_received.velocities.clear();
        geometry_msgs::Twist velocity;
        geometry_msgs::PoseStamped position;

        nav_msgs::Path path_to_publish;
        path_to_publish.header.frame_id = "map";

        for (auto &point : msg->points) {
          position.pose.position.x = point.positions[0];
          position.pose.position.y = point.positions[1];
          position.pose.position.z = point.positions[2];
          position.pose.orientation.x = point.positions[3];
          position.pose.orientation.y = point.positions[4];
          position.pose.orientation.z = point.positions[5];
          position.pose.orientation.w = point.positions[6];

          velocity.linear.x = point.velocities[0];
          velocity.linear.y = point.velocities[1];
          velocity.linear.z = point.velocities[2];
          last_traj_received.velocities.push_back(velocity);
          last_traj_received.positions.push_back(position);
          path_to_publish.poses.push_back(position);
        }
        tracking_pub.publish(path_to_publish);
      };
  auto sub_traj = pnh.subscribe<trajectory_msgs::JointTrajectory>(
      "trajectory_to_follow", 1, trajectoryCallback);
  // ual state subscriber
  auto ualPoseCallback =
      [&uav_state](const geometry_msgs::PoseStamped::ConstPtr &msg) {
        uav_state.current_pose = Eigen::Vector3f(
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        uav_state.current_orientation.x = msg->pose.orientation.x;
        uav_state.current_orientation.y = msg->pose.orientation.y;
        uav_state.current_orientation.z = msg->pose.orientation.z;
        uav_state.current_orientation.w = msg->pose.orientation.w;
      };
  auto pose_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("ual/pose", 1, ualPoseCallback);
  ros::Subscriber ual_pose_sub =
      nh.subscribe<geometry_msgs::PoseStamped>("ual/pose", 1, ualPoseCallback);
  ros::Subscriber ual_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>(
      "ual/velocity", 1, [&](const geometry_msgs::TwistStamped::ConstPtr &msg) {
        uav_state.current_vel = Eigen::Vector3f(
            msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);
      });
  ros::Publisher velocity_ual_pub =
      nh.advertise<geometry_msgs::TwistStamped>("ual/set_velocity", 1);
  grvc::utils::PidController yaw_pid("yaw", YAW_PID_P, YAW_PID_I, YAW_PID_P);
  nh.getParam("trajectory_follower_node/drone_id", drone_id);

  while (ros::ok) {
    ROS_INFO("Drone %d: waiting for trajectory. Pose on path: %d", drone_id,
             pose_on_path);
    // wait for receiving trajectories
    while ((
        !last_traj_received.positions.empty() &&
        !last_traj_received.velocities
             .empty())) {  // if start trajectory is provided by topic or by csv
      pose_on_path =
          cal_pose_on_path(last_traj_received.positions, uav_state.current_pose,
                           pose_on_path);
      ROS_INFO("Drones %d: pose on path: %d", drone_id, pose_on_path);
      target_pose = cal_pose_look_ahead(last_traj_received.positions,
                                        follower.look_ahead, pose_on_path);
      std::cout << "target pose: " << target_pose << std::endl;
      // if the point to go is out of the trajectory, the trajectory will be
      // finished and cleared
      if (target_pose == last_traj_received.positions.size()) {
        ROS_INFO("Drone %d: end of the trajectory", drone_id);
        last_traj_received.positions.clear();
        last_traj_received.velocities.clear();
        pose_on_path = 0;
        break;
      }
      ROS_INFO("Drone %d: look ahead: %d", drone_id, target_pose);
      Eigen::Vector3f pose_to_go = Eigen::Vector3f(
          last_traj_received.positions[target_pose].pose.position.x,
          last_traj_received.positions[target_pose].pose.position.y,
          last_traj_received.positions[target_pose].pose.position.z);
      Eigen::Vector3f vel_to_go;
      if (pose_on_path == 0) {
        Eigen::Vector3f pose_to_go =
            Eigen::Vector3f(last_traj_received.positions[0].pose.position.x,
                            last_traj_received.positions[0].pose.position.y,
                            last_traj_received.positions[0].pose.position.z);
        float dist = (pose_to_go - uav_state.current_pose).norm();
        vel_to_go = Eigen::Vector3f(
            last_traj_received.velocities[pose_on_path].linear.x + dist,
            last_traj_received.velocities[pose_on_path].linear.y + dist,
            last_traj_received.velocities[pose_on_path].linear.z + dist);
      } else
        vel_to_go = Eigen::Vector3f(
            last_traj_received.velocities[pose_on_path].linear.x,
            last_traj_received.velocities[pose_on_path].linear.y,
            last_traj_received.velocities[pose_on_path].linear.z);

      Eigen::Vector3f velocity_to_command =
          calculate_vel(pose_to_go, vel_to_go, uav_state.current_pose);

      // yaw
      tf2::Quaternion desired_q(
          last_traj_received.positions[target_pose].pose.orientation.x,
          last_traj_received.positions[target_pose].pose.orientation.y,
          last_traj_received.positions[target_pose].pose.orientation.z,
          last_traj_received.positions[target_pose].pose.orientation.w);
      tf2::Quaternion current_q(
          uav_state.current_orientation.x, uav_state.current_orientation.y,
          uav_state.current_orientation.z, uav_state.current_orientation.w);

      float current_yaw = tf2::getYaw(current_q);
      float desired_yaw = tf2::getYaw(desired_q);
      float yaw_diff = calculateYawDiff(desired_yaw, current_yaw);
      float sampling_period = rate;

      // publish topic to ual
      geometry_msgs::TwistStamped vel;
      vel.header.frame_id = "map";
      vel.twist.linear.x = velocity_to_command.x();
      vel.twist.linear.y = velocity_to_command.y();
      vel.twist.linear.z = velocity_to_command.z();
      vel.twist.angular.z = yaw_pid.control_signal(yaw_diff, sampling_period);

      velocity_ual_pub.publish(vel);
      ros::spinOnce();
      ros::Duration(rate).sleep();
    }
    ros::spinOnce();
    ros::Duration(1).sleep();
  }

  return 0;
}
