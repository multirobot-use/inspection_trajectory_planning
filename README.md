# Inspection Trajectory Planning
## Overview
This repository is about an operator-oriented inspection task of cylindrical structures, in this case, of wind turbines, with optimal trajectory planning of a multi UAV formation. To generate the optimal trajectory planning, a reference path is given to an optimal solver in order to minimize the accelerations of each drone, making their paths smooth and avoiding jerky movements and going through the optimal path at a constant speed (cruising speed). This also encourages to have a good image taken from the drones, that is intended to build an AR image where some interesting information from the mission is placed.

This work has developed on Ubuntu 18.04 with ROS Melodic and has not been tested in other Ubuntu/ROS versions.

## How to launch simulations and experiments
There are two ways to launch the simulations: simulation-oriented and experiment-oriented; but only one way to launch the experiments in real life: experiment-oriented.

### Simulation-oriented:
Simulation-oriented employs an interface that is mainly intended to be used to debug the code on simulations. Its functionality is based on executing an auto-configuration that takes off the drones, adds some waypoints (depending on the experiment that is going to be launched) and starts the mission. Afterwards, the operator can interact with the formation through it by landing the drones, adding new waypoints, changing the distance to the inspection point or the relative angle, among other things.

Now, there is an overview of what you have to do to make the simulation work adding some notes that may be of your interest:

1. Modify the number of drones that you want to launch on:
* n_drones (possible values: 2 or 3)

    mission_planner/param/mission_planner.yml
    
* f_route --> change the number of the experiment (0 to 4 for two drones, 5 to 9 for three drones, watch the operator's interface oriented to experiments to know the differences or watch the files to know where the waypoints are located):

    mission_planner/script/mission_script.py

* Comment/uncomment the necessary lines to adapt what is going to be launched (2 or 3 drones):

    mission_planner/script/.tmuxinator.yml

2. To make the simulation work:
* Launch a terminal:

    cd your_workspace/src/inspection_trajectory_planning/packages/mission_planner/script

    tmuxinator

**Note**: what are you executing is actually a tmux file. Go through each tab to see what is going to be launched.

### Experiment-oriented:
Experiment-oriented employs a simpler interface regarding the simulation-oriented one. In this case, it has straightforward sequential steps to follow: choose the experiment, take off the UAVs, start the mission, stop the mission, land the UAVs. As expected, this is mainly used to test the correct functionality of the formation in experiments in real life, giving gradually more complex situations. The distribution of files and the way to execute it is completely different from the simulation-oriented as it is thought to have one exclusive PC working for each UAV.

Now, there is an overview of what you have to do to make the simulation work adding some notes that may be of your interest:

1. Modify the number of drones that you want to launch on:
* n_drones (possible values: 2 or 3):

    mission_planner/param/mission_planner.yml

2. To make the simulation work:
* Launch terminal 1 (Gazebo):

    roslaunch mission_planner gazebo_world_2drones.launch (if there are 2 drones chosen)

    roslaunch mission_planner gazebo_world_3drones.launch (if there are 3 drones chosen)

* Launch terminal 2 (Operator's interface --> experiment oriented):

    cd your_workspace/src/inspection_trajectory_planning/packages/mission_planner/script

    python experiments.py

* Launch terminal 3 (Independent leader drone):

    your_workspace/src/inspection_trajectory_planning/packages/mission_planner/launch/experiments/leader

    tmuxinator
    
* Launch terminal 4 (Independent follower1 drone):

    your_workspace/src/inspection_trajectory_planning/packages/mission_planner/launch/experiments/follower1

    tmuxinator
    
* Launch terminal 5 (Independent follower2 drone) (ONLY IF 3 DRONES WERE CHOSEN):

    your_workspace/src/inspection_trajectory_planning/packages/mission_planner/launch/experiments/follower2

    tmuxinator

#### Additional information
There are some parameters that can be of your interest to modify, for instance, the cruising speed of the UAVs, the step size, the number of steps of the receding horizon, the increasing amount of relative angle or distance of the joystick, among others. Those things can be changed on:

    mission_planner/param/mission_planner.yml

