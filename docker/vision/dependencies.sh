#!/bin/sh
set -e
ROS_DISTRO="noetic"

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
        yasm libatlas-base-dev libpq-dev \
        python3-pip python3-numpy tzdata \
        \
        cmake gdb git libgtk2.0-dev pkg-config \
        python-numpy \
        qtdeclarative5-dev python3-pip zip \
        \
        tensorrt \
        \
        ros-"$ROS_DISTRO"-ros-base usbutils libusb-1.0.0-dev

if [ "$ROS_DISTRO" = "noetic" ]; then
    apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool
fi
