HOW TO LAUNCH SIMULATIONS AND EXPERIMENTS (may have slight changes yet):

================================================================================================
SIMULATION-ORIENTED:

1. Modify the number of drones that you want to launch on:
    mission_planner/param/mission_planner.yml
        n_drones (possible values: 2 or 3)
    mission_planner/script/mission_script.py
        f_route --> change the number of the experiment (0 to 4 for two drones, 5 to 9 for
        three drones, watch the operator's interface oriented to experiments to know the
        differences or watch the files to know where the waypoints are located)
    mission_planner/script/.tmuxinator.yml
        Comment/uncomment the necessary lines to adapt what is going to be launched (2 or 
        3 drones)

2. To make the simulation work:
    Launch a terminal
        cd your_workspace/src/inspection_trajectory_planning/packages/mission_planner/script
        tmuxinator


================================================================================================
EXPERIMENT-ORIENTED:

1. Modify the number of drones that you want to launch on:
    mission_planner/param/mission_planner.yml
        n_drones (possible values: 2 or 3)

2. To make the simulation work:
    Launch terminal 1 (Gazebo):
        roslaunch mission_planner gazebo_world_2drones.launch (if there are 2 drones chosen)
        roslaunch mission_planner gazebo_world_3drones.launch (if there are 3 drones chosen)

    Launch terminal 2 (Operator's interface --> experiment oriented):
        cd your_workspace/src/inspection_trajectory_planning/packages/mission_planner/script
        python experiments.py

    Launch terminal 3 (Independent leader drone):
        your_workspace/src/inspection_trajectory_planning/packages/mission_planner/launch/experiments/leader
        tmuxinator
    
    Launch terminal 4 (Independent follower1 drone):
        your_workspace/src/inspection_trajectory_planning/packages/mission_planner/launch/experiments/follower1
        tmuxinator
    
    Launch terminal 5 (Independent follower2 drone) (ONLY IF 3 DRONES WERE CHOSEN):
        your_workspace/src/inspection_trajectory_planning/packages/mission_planner/launch/experiments/follower1
        tmuxinator
