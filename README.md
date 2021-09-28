# Inspection Trajectory Planning
## Overview
This repository is about an operator-oriented inspection task of cylindrical structures, in this case, of wind turbines, with optimal trajectory planning of a multi UAV formation. To generate the optimal trajectory planning, a reference path is given to an optimal solver in order to minimize the accelerations of each drone, making their paths smooth and avoiding jerky movements and going through the optimal path at a constant speed (cruising speed). This also encourages to have a good image taken from the drones, that is intended to build an AR image where some interesting information from the mission.

This work has developed on Ubuntu 18.04 with ROS Melodic and does not have been tested in other Ubuntu/ROS versions.

## Manual installation
To install the repositories correctly, you have to follow the next steps:

1. Create a workspace

```
cd ~
mkdir your_ws
cd your_ws
mkdir src
cd src
```

2. Install the inspection_trajectory_planning repository

```
git clone https://github.com/grvcTeam/inspection_trajectory_planning
cd inspection_trajectory_planning/packages
```

3. Clone the necessary packages, which are: ACADO, safe_corridor, catkin_simple, grvc-ual, seeker-ros, grvc-utils

```
git clone https://github.com/alfalcmar/acado.git
git clone (link repo safe_corridor NOT AVAILABLE YET)
git clone https://github.com/catkin/catkin_simple
git clone https://github.com/grvcTeam/grvc-ual
git clone https://github.com/aguramos93/seeker-ros
git clone https://github.com/grvcTeam/grvc-utils
```

**Note**: you may not have installed the following packages:

Catkin tools: https://catkin-tools.readthedocs.io/en/latest/installing.html

```
sudo apt install tmuxinator
sudo pip install pynput
```

4. Install safe_corridor_generator packages

```
cd ~/your_ws/src/inspection_trajectory_planning/packages/safe_corridor_generator/thirdparty/jps3d
mkdir -p build
cd build
cmake ..
make
cd ../..
cd DecompROS/DecompUtil
mkdir -p build
cd build
cmake ..
make
```

5. Install ACADO

https://acado.github.io/install_linux.html

```
sudo apt-get install gcc g++ cmake git gnuplot doxygen graphviz
cd ~/your_ws/src/inspection_trajectory_planning/packages/acado
mkdir -p build
cd build
cmake ..
make
echo "source ~/your_ws/src/inspection_trajectory_planning/packages/acado/build/acado_env.sh" >> ~/.bashrc
```

6. Configure and setup UAL. Only MAVROS needed. Install dependencies

https://github.com/grvcTeam/grvc-ual/wiki/How-to-build-and-install-grvc-ual

```
cd ~/your_ws/src/inspection_trajectory_planning/packages/grvc-ual
./configure.py
```

7. Install dependencies of grvc-ual

```
sudo apt-get install libeigen3-dev ros-$(rosversion -d)-geodesy ros-$(rosversion -d)-joy
```

8. MAVROS

```
sudo apt install -y ros-$(rosversion -d)-mavros ros-$(rosversion -d)-mavros-extras
sudo geographiclib-get-geoids egm96-5
sudo usermod -a -G dialout $USER
sudo apt remove modemmanager
```

9. PX4 SITL Simulations

https://github.com/grvcTeam/grvc-ual/wiki/Setup-instructions:-PX4-SITL

```
sudo apt install libgstreamer1.0-dev python-jinja2 python-pip
pip install numpy toml
cd ~/your_ws/src/inspection_trajectory_planning/packages
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout v1.10.2
git submodule update --init --recursive
make
make px4_sitl_default gazebo
```

10. Execute install.sh

```
cd ~/your_ws/src/inspection_trajectory_planning
./install.sh
```

## Automatic installation
There is a .sh file available to install the necessary packages automatically, except for the safe_corridor package. An interface for grvc-ual package will appear in the middle of the installation, it is only necessary to select MAVROS and the dependencies.

```
cd ~/your_ws/src/inspection_trajectory_planning/
./install_packages.sh
./install.sh
```


## How to launch simulations and experiments
There are two ways to launch the simulations: simulation-oriented and experiment-oriented; but only one way to launch the experiments in real life: experiment-oriented.

### Simulation-oriented:
Simulation-oriented employs an interface that is mainly intended to be used to debug the code on simulations. Its functionality is based on executing an auto-configuration that take off the drones, add some waypoints (depending on the experiment that is going to be launched) and starts the mission. Afterwards, the operator can interact with the formation through it by landing the drones, adding new waypoints, changing the distance to the inspection point or the relative angle, among other things.

Now, there is an overview of what you have to do to make the simulation work adding some notes that may be of your interest:

1. Modify the number of drones that you want to launch on:
    * mission_planner/param/mission_planner.yml
        n_drones (possible values: 2 or 3)

    * mission_planner/script/mission_script.py
        f_route --> change the number of the experiment (0 to 4 for two drones, 5 to 9 for
        three drones, watch the operator's interface oriented to experiments to know the
        differences or watch the files to know where the waypoints are located)

    * mission_planner/script/.tmuxinator.yml
        Comment/uncomment the necessary lines to adapt what is going to be launched (2 or 
        3 drones)

2. To make the simulation work:
Launch a terminal

```
    cd your_workspace/src/inspection_trajectory_planning/packages/mission_planner/script
    tmuxinator
```

**Note**: what you are executing is actually a tmux file. Go through each tab to see what is going to be launched.

### Experiment-oriented:
Experiment-oriented employs an simpler interface regarding the simulation-oriented one. In this case, it has a straightforward sequential steps to follow: choose the experiment, take off the UAVs, start the mission, stop the mission, land the UAVs. As expected, this is mainly used to test the correct functionality of the formation on experiments in real life, giving gradually more complex situations. The distribution of files and the way to execute it is completely different from the simulation-oriented as it is thought to have one exclusive PC working for each UAV.

Now, there is an overview of what you have to do to make the simulation work adding some notes that may be of your interest:

1. Modify the number of drones that you want to launch on:
```
mission_planner/param/mission_planner.yml
            n_drones (possible values: 2 or 3)
```

2. To make the simulation work:

Launch terminal 1 (Gazebo):

```
    roslaunch mission_planner gazebo_world_2drones.launch
```
```
    roslaunch mission_planner gazebo_world_3drones.launch
```

Launch terminal 2 (Operator's interface --> experiment oriented):

```
    cd your_workspace/src/inspection_trajectory_planning/packages/mission_planner/script
    python experiments.py
```

Launch terminal 3 (Independent leader drone):

```
    your_workspace/src/inspection_trajectory_planning/packages/mission_planner/launch/experiments/leader
    tmuxinator
```
    
Launch terminal 4 (Independent follower1 drone):

```
    your_workspace/src/inspection_trajectory_planning/packages/mission_planner/launch/experiments/follower1
    tmuxinator
```
    
Launch terminal 5 (Independent follower2 drone) (Only if 3 drones were chosen):

```
    your_workspace/src/inspection_trajectory_planning/packages/mission_planner/launch/experiments/follower2
    tmuxinator
```

#### Additional information
There are some parameters that can be of your interest to modify, for instance, the cruising speed of the UAVs, the step size, the number of steps of the receding horizon, the increasing amount of relative angle or distance of the joystick, among others. Those things can be changed on:

    mission_planner/param/mission_planner.yml

