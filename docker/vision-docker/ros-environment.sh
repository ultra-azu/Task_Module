#!/bin/bash
set -e 
ROS_DISTRO="noetic"

echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash

pushd src
git clone --recursive https://github.com/Rumarino-Team/Vision-Zed-Ros-Wrapper.git
cd Vision-Zed-Ros-Wrapper
git am /0001-WIP-Add-missing-package-dependencies.patch
popd 
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
catkin_make -DCMAKE_BUILD_TYPE=RELEASE -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.8
source ./devel/setup.bash