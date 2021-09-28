#!/bin/bash

## Get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"
WORKSPACE_PATH="$( cd .. ; pwd -P )"


## Install necessary packages
sudo apt-get install sudo apt-get install libeigen3-dev ros-$(rosversion -d)-geodesy ros-$(rosversion -d)-joy
sudo apt install tmuxinator
sudo pip install pynput


## Clone packages
echo "Cloning necessary packages"
cd $WORKSPACE_PATH/inspection_trajectory_planning/packages

git clone https://github.com/alfalcmar/acado.git
# git clone (link repo safe_corridor) # Pending
git clone https://github.com/catkin/catkin_simple
git clone https://github.com/grvcTeam/grvc-ual
git clone https://github.com/aguramos93/seeker-ros


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
# echo "Installing safe_corridor"
# cd $WORKSPACE_PATH/inspection_trajectory_planning/packages/safe_corridor_generator/thirdparty/jps3d
# mkdir -p build
# cd build
# cmake ..
# make
# cd ../..
# cd DecompROS/DecompUtil
# mkdir -p build
# cd build
# cmake ..
# make


## Install and configure UAL. Only MAVROS needed. Dependencies
echo "Installing and configuring UAL. Only MAVROS needed. Install dependencies"
cd $WORKSPACE_PATH/inspection_trajectory_planning/packages/grvc-ual
./configure.py


## Install MAVROS packages
echo "Installing MAVROS necessary packages"
sudo apt install -y ros-$(rosversion -d)-mavros ros-$(rosversion -d)-mavros-extras
sudo geographiclib-get-geoids egm96-5
sudo usermod -a -G dialout $USER
sudo apt remove modemmanager


## Install PX4 for SITL simulations
echo "Installing PX4 for SITL simulations"
sudo apt install libgstreamer1.0-dev python-jinja2 python-pip
pip install numpy toml
cd $WORKSPACE_PATH/inspection_trajectory_planning/packages
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout v1.10.2
git submodule update --init --recursive
make
make px4_sitl_default gazebo
