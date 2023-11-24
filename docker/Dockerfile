FROM osrf/ros:melodic-desktop-full

# Install dependencies
RUN apt update && apt install -y\
 ros-melodic-smach\
 ros-melodic-smach-ros\
 ros-melodic-executive-smach\
 ros-melodic-smach-viewer\
 ros-melodic-moveit\
 ros-melodic-moveit-visual-tools\
 vim\
 nano\
 python-rosdep\
 python-rosinstall\
 python-rosinstall-generator\
 python-wstool\
 build-essential\
 ros-melodic-catkin\
 python-catkin-tools\
 python-scipy

# Update dependencies
RUN rosdep update
RUN apt-get update && apt-get dist-upgrade

# Create a Catkin Workspace
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN . /opt/ros/melodic/setup.sh && \
 catkin_make

# Download ROS packages
WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/uuvsimulator/uuv_simulator.git
RUN git clone https://github.com/uuvsimulator/desistek_saga.git
RUN git clone https://github.com/uuvsimulator/rexrov2.git

# Install packages and their dependencies
WORKDIR /root/catkin_ws
RUN . /opt/ros/melodic/setup.sh && \
 . /root/catkin_ws/devel/setup.sh && \
 catkin_make install

# Add setup.sh for catkin workspace to bashrc
RUN echo "source /root/catkin_ws/devel/setup.sh" >> ~/.bashrc
