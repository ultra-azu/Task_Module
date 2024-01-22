# TODO: Put important dependencies of embedded  in here
# This image at the end of compiling the ROS codes it will
# delete the ros depedenciies inside /opt/ros/noetic
# Because it will share it with the Vision image with a volume.



# This Dockerfile is for building the Arduino image for the Rosserial Arduino Library.
#  From the Hydrus_Embedded repository: https://github.com/Cruiz102/Hydrus_Embedded
FROM ros:noetic-ros-base
# Set the 
ENV ARDUINO_BOARD="arduino:avr:uno"



RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       gcc \
       curl \
       vim \
       git  && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /usr/local/
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh |  sh  &&\
    arduino-cli core update-index &&\
    arduino-cli core install arduino:avr



# Installing the latest version of the Rosserial Arduino Library 0.9.4 got me the following error:
#  Therefore I had to install the version 0.7.9 as mentioned in this post.
# https://answers.ros.org/question/361930/rosserial-arduino-compilation-error-no-cstring/

RUN arduino-cli lib install "Rosserial Arduino Library@0.7.9" &&\
    sed -i '/#include "ros\/node_handle.h"/a #include "geometry_msgs/Vector3.h"' /root/Arduino/libraries/Rosserial_Arduino_Library/src/ros.h

RUN arduino-cli lib  install "Servo@1.2.1"
RUN arduino-cli lib install "BlueRobotics MS5837 Library@1.1.1"
WORKDIR /root/Arduino/libraries
# Download the Hydrus_Embedded library from github
RUN git clone https://github.com/Cruiz102/Hydrus_Embedded.git

# Delete the ros depedencies inside /opt/ros/noetic
# Because it will share it with the Vision image with a volume.
RUN rm -rf /opt/ros/noetic

WORKDIR /root/Arduino/libraries/Hydrus_Embedded/examples
# CMD [ "arduino-cli compile --fqbn $ARDUINO_BOARD Hydrus.ino" ]