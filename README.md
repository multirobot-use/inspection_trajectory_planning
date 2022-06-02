# Inspection Trajectory Planning
## Overview
This repository is about an operator-oriented inspection task of cylindrical structures, in this case, of wind turbines, with optimal trajectory planning of a multi UAV formation. To generate the optimal trajectory planning, a reference path is given to an optimal solver in order to minimize the accelerations of each drone, making their paths smooth and avoiding jerky movements and going through the optimal path at a constant speed (cruising speed). This also encourages to have a good image taken from the drones, that is intended to build an AR image where some interesting information from the mission in the early future.

This work is developed on Ubuntu 18.04 with ROS Melodic and does not have been tested in other Ubuntu/ROS versions.

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

2. Install the *inspection_trajectory_planning* repository

```
git clone https://github.com/grvcTeam/inspection_trajectory_planning
```

3. Initialize and update submodules

```
cd ~/your_ws/src/inspection_trajectory_planning
git submodule update --init
```

4. Clone the necessary packages, which are: *ACADO*, *safe_corridor*, *catkin_simple*, *grvc-ual*, *seeker-ros*, *grvc-utils*

```
cd ~/your_ws/src/inspection_trajectory_planning/packages
git clone https://github.com/alfalcmar/acado.git
git clone https://github.com/catkin/catkin_simple
git clone https://github.com/grvcTeam/grvc-ual
git clone https://github.com/aguramos93/seeker-ros
git clone https://github.com/grvcTeam/grvc-utils
```

**Note**: you may not have installed the following packages:

Catkin tools: https://catkin-tools.readthedocs.io/en/latest/installing.html

```
sudo apt install tmuxinator
pip install pynput
sudo apt install xz-utils
sudo apt-get install python3-catkin-tools
```

5. Install *safe_corridor_generator* packages

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

6. Install *ACADO*

https://acado.github.io/install_linux.html

```
sudo apt-get install gcc g++ cmake git gnuplot doxygen graphviz
cd ~/your_ws/src/inspection_trajectory_planning/packages/acado
mkdir -p build
cd build
cmake ..
make
```

**Note**: add the source to the .bashrc or in the .zshrc

```
echo "source ~/your_ws/src/inspection_trajectory_planning/packages/acado/build/acado_env.sh" >> ~/.bashrc
echo "source ~/your_ws/src/inspection_trajectory_planning/packages/acado/build/acado_env.sh" >> ~/.zshrc
```


7. Configure and setup *UAL*. Only *MAVROS* and *Gazebo Light* needed. Install dependencies

https://github.com/grvcTeam/grvc-ual/wiki/How-to-build-and-install-grvc-ual

```
cd ~/your_ws/src/inspection_trajectory_planning/packages/grvc-ual
./configure.py
```

8. Install dependencies of *grvc-ual*

```
sudo apt-get install libeigen3-dev ros-$(rosversion -d)-geodesy ros-$(rosversion -d)-joy
```

9. *MAVROS*

```
sudo apt install -y ros-$(rosversion -d)-mavros ros-$(rosversion -d)-mavros-extras
sudo geographiclib-get-geoids egm96-5
sudo usermod -a -G dialout $USER
sudo apt remove modemmanager
```

10. *PX4* SITL Simulations

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

11. Build and source

```
cd ~/your_ws/
catkin build
```

**Note**: add the source to the .bashrc or in the .zshrc

```
echo "source ~/your_ws/devel/setup.bash" >> ~/.bashrc
echo "source ~/your_ws/devel/setup.zsh" >> ~/.zshrc
```


## Automatic installation
There is a .sh file available to install the necessary packages automatically (including ROS Melodic). An interface for *grvc-ual* package will appear in the middle of the installation, it is only necessary to select *MAVROS*, *Gazebo Light* and the dependencies.

```
cd ~
mkdir -p inspection_ws
cd inspection_ws
mkdir -p src
cd src
git clone https://github.com/grvcTeam/inspection_trajectory_planning
cd inspection_trajectory_planning
./install_packages.sh
```

## Test to verify everything is installed correctly
In order to ensure that the installation has been completed successfully, you have to run a simple test of the mission, only executing one UAV in a simulation-oriented environment.

First, modify the parameter *n_drones*, which represents the number of drones which the mission will operate with, and put this value to 1. You can find this parameter in the file *mission_planner.yml* which has several parameters concerning to the mission.

```
packages/mission_planner/param/mission_planner.yml
```

Second, we have to launch the session file that is on the folder *packages/mission_planner/session_files/session_1drone.yml*. 

**Note**: If you do not know what is tmuxinator, it allows to create and manage tmux sessions easily. This way, we can execute several launch files in the same terminal at the same time. To do that, in this case:

```
cd packages/mission_planner/
./start1.sh
```

**Note**: It is important to mention that, if you want to change the tab of the tmux, you have to press Ctrl + B + *number of desired tab*. Also, if you want to close the whole tmux session, you have to put *tmux kill-server* on any of the tabs or terminal.


## How to launch simulations and experiments
There are two ways to launch the simulations: simulation-oriented and experiment-oriented; but only one way to launch the experiments in real life: experiment-oriented.

### Simulation-oriented:
Simulation-oriented employs an interface that is mainly intended to be used to debug the code on simulations. Its functionality is based on executing an auto-configuration file that take off the drones, add some waypoints (depending on the experiment that is going to be launched) and starts the mission. Afterwards, the operator can interact with the formation through it by landing the drones, adding new waypoints, changing the distance to the inspection point or the relative angle, among other things.

Now, we show an overview of what you have to do to make the simulation work, adding some notes that may be of your interest:

1. Modify the number of drones that you want to launch on:
    * mission_planner/param/mission_planner.yml
        n_drones (possible values: 1, 2 or 3)

    * mission_planner/script/operator_interface.py
        f_route --> change the name file depending on the number of drones that you want to
        deploy: exp_1drones.yml, exp_2drones.yml or exp_3drones.yml. Modify it to test

    * mission_planner/script/.tmuxinator.yml
        Comment/uncomment the necessary lines to adapt what is going to be launched (2 or 
        3 drones)

2. To make the simulation work:
Launch a terminal

```
    cd your_workspace/src/inspection_trajectory_planning/packages/mission_planner/
    ./startX.sh
```

**Note**: what you are executing is actually a tmux file. Go through each tab using Ctrl + B + *number of desired tab* to see what is being launched. Remember that you can finish the session by putting *tmux kill-server* on any tab or terminal.

### Experiment-oriented:
Experiment-oriented employs a simpler interface regarding the simulation-oriented one. In this case, it has a straightforward sequential steps to follow: choose the experiment, take off the UAVs, start the mission, stop the mission, land the UAVs. As expected, this is mainly used to test the correct functionality of the formation on experiments in real life, giving gradually more complex situations. The distribution of files and the way to execute it is completely different from the simulation-oriented as it is thought to have one exclusive PC working for each UAV.

Now, we show an overview of what you have to do to make the simulation work, adding some notes that may be of your interest:

1. Modify the number of drones that you want to launch on:
```
mission_planner/param/mission_planner.yml
            n_drones (possible values: 2 or 3)
```

2. Modify the yml file that is being read in the *operator_interface.py*:
* mission_planner/script/operator_interface.py
        f_route --> change the number of the experiment (0 to 4 for two drones, 5 to 9 for
        three drones, watch the operator's interface oriented to experiments to know the
        differences or watch the files to know where the waypoints are located)

3. To make the simulation work:

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
    python3 operator_interface.py
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
There are some parameters that can be of your interest to modify, for instance, the cruising speed of the UAVs, the step size, the number of steps of the receding horizon, the increasing amount of relative angle or distance of the joystick, among others. Those things can be changed in the file:

```
packages/mission_planner/param/mission_planner.yml
```


### Frequent issues
In case that the installation did not go well, try to reinstall. Sometimes, a single package may not be installed correctly and, in case of doubt, it is best to reinstall everything.

Once the repository is correctly installed, it is time to build the packages using *catkin build*. Sometimes, you may notice that your PC freezes in the process. If that occurs, try to reboot your PC and put *catkin build* again. If even in that case freezes, use *catkin clean* and then *catkin build*, maybe twice.

When the workspace has been built, the first try of running a simulation may not work: Gazebo does not start and, if that happens, there is nothing to see. Kill the tmuxinator server by putting on any tab of tmux *tmux kill-server* and try to run it again.

If the formation has taken off and they do not generate trajectories, make sure that the mission has started correctly on the operator interface (HMI) and see if the parameter *n_drones* is the correct one as well.