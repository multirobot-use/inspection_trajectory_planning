#include "mission_planner_ros.hpp"

int main(int argc, char **argv) {
  // Initializes ROS, and sets up a node
  ros::init(argc, argv, "mission_planner");
  ros::NodeHandle nh("~");

  bool leader = true;
  trajectory_planner::safeGetParam(nh, "leader", leader);
  MissionPlannerRos MissionPlannerRos(nh, leader);

  ros::spin();
  return 0;
}