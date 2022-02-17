%% Load ROS experiments
% https://es.mathworks.com/help/ros/ref/rosbag.html
clc; clear all; close all;

cd ~/paper
experiment_date = "obs_avoidance_1"; % For saving plots
% bagfile = rosbag("simulations/exp5_drone_1_experiment_2021-07-28*");
% bagfile = rosbag("simulations/exp5_drone_2_experiment_2021-07-28*");
% bagfile = rosbag('changing_references/simulation_2022-01-31-08-56-09*');
bagfile = rosbag('obstacle_avoidance/good_one/simulation_2022-01-31-11-42-26*');

n_drones = 3;

start_time_1 = bagfile.StartTime;
end_time_1   = bagfile.EndTime;
start_time_2 = bagfile.StartTime;
end_time_2   = bagfile.EndTime;
if n_drones == 3
    start_time_3 = bagfile.StartTime;
    end_time_3   = bagfile.EndTime;
end

% Name of topics
if n_drones == 2
    topic_traj_to_follow = ["/drone_1/follower/trajectory_to_follow", "/drone_2/follower/trajectory_to_follow"];
    topic_points_to_inspect = ["/drone_1/mission_planner_ros/points_to_inspect_transformed", "/drone_2/mission_planner_ros/points_to_inspect_transformed"];
    topic_ref_traj = ["/drone_1/mission_planner_ros/ref_traj", "/drone_2/mission_planner_ros/ref_traj"];
    topic_solved_traj = ["/drone_1/mission_planner_ros/solved_traj", "/drone_2/mission_planner_ros/solved_traj"];
    topic_pose = ["/drone_1/ual/pose", "/drone_2/ual/pose"];
    topic_velocity = ["/drone_1/ual/velocity", "/drone_2/ual/velocity"];
%     topic_camera = ["/drone_1/cgo3_camera/image_raw/compressed", "/drone_2/cgo3_camera/image_raw/compressed"];
    topic_distance_to_inspect = ["/drone_1/absolute_distance_to_inspect", "/drone_2/absolute_distance_to_inspect"];
    topic_inspection_distance = ["/drone_1/inspection_distance", "/drone_2/inspection_distance"];
    topic_relative_angle = ["/drone_1/absolute_relative_angle", "/drone_2/absolute_relative_angle"];
    topic_formation_angle = ["/drone_2/formation_angle"];
    topic_mission_status = ["/drone_1/mission_status", "/drone_2/mission_status"];
else % n_drones = 3
    topic_traj_to_follow = ["/drone_1/follower/trajectory_to_follow", "/drone_2/follower/trajectory_to_follow", "/drone_3/follower/trajectory_to_follow"];
    topic_points_to_inspect = ["/drone_1/mission_planner_ros/points_to_inspect_transformed", "/drone_2/mission_planner_ros/points_to_inspect_transformed", "/drone_3/mission_planner_ros/points_to_inspect_transformed"];
    topic_ref_traj = ["/drone_1/mission_planner_ros/ref_traj", "/drone_2/mission_planner_ros/ref_traj", "/drone_3/mission_planner_ros/ref_traj"];
    topic_solved_traj = ["/drone_1/mission_planner_ros/solved_traj", "/drone_2/mission_planner_ros/solved_traj", "/drone_3/mission_planner_ros/solved_traj"];
    topic_pose = ["/drone_1/ual/pose", "/drone_2/ual/pose", "/drone_3/ual/pose"];
    topic_velocity = ["/drone_1/ual/velocity", "/drone_2/ual/velocity", "/drone_3/ual/velocity"];
%     topic_camera = ["/drone_1/cgo3_camera/image_raw/compressed", "/drone_2/cgo3_camera/image_raw/compressed", "/drone_3/cgo3_camera/image_raw/compressed"];
    topic_distance_to_inspect = ["/drone_1/absolute_distance_to_inspect", "/drone_2/absolute_distance_to_inspect", "/drone_3/absolute_distance_to_inspect"];
    topic_inspection_distance = ["/drone_1/inspection_distance", "/drone_2/inspection_distance", "/drone_3/inspection_distance"];
    topic_relative_angle = ["/drone_1/absolute_relative_angle", "/drone_2/absolute_relative_angle", "/drone_3/absolute_relative_angle"];
    topic_formation_angle = ["/drone_2/formation_angle", "/drone_3/formation_angle"];
    topic_mission_status = ["/drone_1/mission_status", "/drone_2/mission_status", "/drone_3/mission_status"];
end

% Dividing in different scripts in order to not overload the main file
% Read all the topics from the rosbag file
reading_topics;

% Show plots and autosave
save_plots = 1; % 0 to not save, 1 to save
% plot_figures_experiments;
% plot_obstacle_avoidance;
plot_figures_experiments_obs_avoidance;

