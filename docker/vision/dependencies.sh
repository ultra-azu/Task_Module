#!/bin/sh
set -e
ROS_DISTRO="noetic"

# Print Ubuntu version
apt-get update && apt-get install -y lsb-release

# ROS preparation
apt-get install -y curl
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update

# Install build tools, build dependencies and python
# Not sure which dependency is for which, but this is separated based on the
# original Dockerfile
DEBIAN_FRONTEND=noninteractive apt-get install -y \
        build-essential gcc g++ \
	cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
	libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
        yasm libatlas-base-dev gfortran libpq-dev \
        libxine2-dev libglew-dev libtiff5-dev zlib1g-dev libavutil-dev libpostproc-dev \
        libeigen3-dev python3-dev python3-pip python3-numpy libx11-dev tzdata \
        \
        cmake gdb git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
        python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev \
        libdc1394-22-dev qtbase5-dev qtdeclarative5-dev python3-pip zip \
        \
        tensorrt cuda \
        \
        ros-"$ROS_DISTRO"-desktop-full usbutils libusb-1.0.0-dev

if [ "$ROS_DISTRO" = "noetic" ]; then
    apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool
fi

#Tensorrt-inference dependencies maybe?
python3 -m pip install -U pip -i https://pypi.tuna.tsinghua.edu.cn/simple
pip3 install torch torchvision mxnet-cu102 onnx-simplifier -i https://pypi.tuna.tsinghua.edu.cn/simple
pip3 install ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple
pip3 install --ignore-installed -U PyYAML -i https://pypi.tuna.tsinghua.edu.cn/simple
