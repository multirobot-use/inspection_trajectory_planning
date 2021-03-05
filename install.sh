#!/bin/bash

# get path to script
SCRIPT_PATH="$( cd "$(dirname "$0")" ; pwd -P )"
WORKSPACE_PATH="$( cd .. ; pwd -P )"

sudo pip3 install gitman
mkdir -p packages
## pull stable commits of naki repositories
echo "gitman install: cloning packages"
gitman install

## install acado
cd packages/acado
mkdir build
cd build
cmake ..
make

#install px4
TEMP=`(cd "$SCRIPT_PATH/packages/PX4" && pwd)`
cd $TEMP
make
make px4_sitl_default


# build all naki packages
echo "building packages"
catkin build

cd $WORKSPACE_PATH
cd ..
WORKSPACE_PATH="$(  pwd -P )"
echo $WORKSPACE_PATH


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
