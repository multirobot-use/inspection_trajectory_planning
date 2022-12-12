#!/bin/bash

## Get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"
WORKSPACE_PATH="$( cd .. ; pwd -P )"


## ROS noetic installation
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# set bashrc
num=`cat ~/.bashrc | grep "$WORKSPACE_PATH" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "
  source /opt/ros/noetic/setup.bash" >> ~/.bashrc
fi

# set zshrc
num=`cat ~/.zshrc | grep "$WORKSPACE_PATH" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "
  source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
fi

source ~/.bashrc
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update


## Update Gazebo
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/gazebo-stable.list'
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys D2486D2DD83DB69272AFE98867170598AF249743
sudo apt update
sudo apt upgrade


## Catkin tools
sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install -y python3-catkin-tools


## Install necessary packages
sudo apt-get install -y sudo apt-get install libeigen3-dev ros-$(rosversion -d)-geodesy ros-$(rosversion -d)-joy
sudo apt-get install ros-$(rosversion -d)-rosbridge-server
sudo apt install -y tmuxinator

# set bashrc
num=`cat ~/.bashrc | grep "$WORKSPACE_PATH" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "
  alias kmux='tmux kill-server'" >> ~/.bashrc 
fi

# set zshrc
num=`cat ~/.zshrc | grep "$WORKSPACE_PATH" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "
  alias kmux='tmux kill-server'" >> ~/.zshrc 
fi

sudo pip3 install pynput
sudo pip3 install gitman
sudo apt install -y xz-utils


## Submodule init & update
cd $WORKSPACE_PATH/inspection_trajectory_planning
git submodule update --init


## Clone packages
echo "Cloning necessary packages"
cd $WORKSPACE_PATH/inspection_trajectory_planning/packages

git clone https://github.com/alfalcmar/acado.git
git clone https://github.com/catkin/catkin_simple
git clone https://github.com/grvcTeam/grvc-ual
git clone https://github.com/aguramos93/seeker-ros
git clone https://github.com/grvcTeam/grvc-utils
git clone https://github.com/siemens/ros-sharp
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint


## Install acado
echo "Installing acado"
cd $WORKSPACE_PATH/inspection_trajectory_planning/packages/acado
mkdir -p build
cd build
cmake ..
make

# set bashrc
num=`cat ~/.bashrc | grep "$WORKSPACE_PATH" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "
source $WORKSPACE_PATH/inspection_trajectory_planning/packages/acado/build/acado_env.sh" >> ~/.bashrc
fi

# set zshrc
num=`cat ~/.zshrc | grep "$WORKSPACE_PATH" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "
source $WORKSPACE_PATH/inspection_trajectory_planning/packages/acado/build/acado_env.sh" >> ~/.zshrc
fi


## Install safe_corridor (uncomment if the package is downloaded)
echo "Installing safe_corridor"
cd $WORKSPACE_PATH/inspection_trajectory_planning/packages/safe_corridor_generator/thirdparty/jps3d 
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


## Install and configure UAL. Only MAVROS and Gazebo Light needed. Dependencies
echo "Installing and configuring UAL. Only MAVROS needed. Install dependencies"
cd $WORKSPACE_PATH/inspection_trajectory_planning/packages/grvc-ual

### NEED TO CHANGE TO NOETIC BRANCH
git checkout -b origin/noetic

./configure.py

## Need to install it TWICE
./configure.py

## Install MAVROS packages
echo "Installing MAVROS necessary packages"
sudo apt install -y ros-$(rosversion -d)-mavros ros-$(rosversion -d)-mavros-extras
sudo geographiclib-get-geoids egm96-5
sudo usermod -a -G dialout $USER
sudo apt remove modemmanager


## Set the ROS environment variables
# set bashrc
num=`cat ~/.bashrc | grep "$WORKSPACE_PATH" | wc -l`
if [ "$num" -lt "1" ]; then

  # set bashrc
  echo "
source $WORKSPACE_PATH/devel/setup.bash" >> ~/.bashrc
  
fi

# set zshrc
num=`cat ~/.zshrc | grep "$WORKSPACE_PATH" | wc -l`
if [ "$num" -lt "1" ]; then

  echo "
source $WORKSPACE_PATH/devel/setup.zsh" >> ~/.zshrc

fi


## Install PX4 for SITL simulations
### NOTE: PX4 compiles, but does not work on Ubuntu 20/ROS Noetic
echo "Installing PX4 for SITL simulations"
sudo apt update
sudo apt install -y libgstreamer1.0-dev python3-jinja2 python3-pip
pip3 install numpy toml
cd $WORKSPACE_PATH/inspection_trajectory_planning/packages
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout v1.10.2
git submodule update --init --recursive
make
make px4_sitl_default gazebo
