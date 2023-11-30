FROM osrf/ros:noetic-desktop-full

# Set non-interactive mode for apt
ENV DEBIAN_FRONTEND=noninteractive

# Install smach and smach_viewer
# Update and install dependencies
RUN apt update && apt install -y \
 ros-noetic-smach-ros \
 ros-noetic-executive-smach \
 ros-noetic-smach-viewer \
 vim\
 git


RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && \
    mkdir -p /home/ubuntu/catkin_ws/src && \
    cd /home/ubuntu/catkin_ws/ && \
    catkin_make && \
    source devel/setup.bash'

# Install PIP
RUN apt update && \
    apt install python3-pip -y && \
    rm -rf /var/lib/apt/lists/*



# Install Sensors Dependencies
WORKDIR /home/ubuntu/catkin_ws/src
RUN git clone https://github.com/stereolabs/zed-ros-interfaces.git  && \
    git clone https://github.com/Rumarino-Team/Hydrus.git &&\
    git clone https://github.com/Rumarino-Team/Task_Module.git 

WORKDIR /home/ubuntu/catkin_ws

RUN rosdep install --from-paths src --ignore-src -r -y