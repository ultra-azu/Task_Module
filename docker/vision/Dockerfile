# Build arguments  
ARG UBUNTU_RELEASE_YEAR=20
ARG ZED_SDK_MAJOR=4
ARG ZED_SDK_MINOR=0
ARG CUDA_MAJOR=11
ARG CUDA_MINOR=8

# Specify the parent image from which we build
FROM stereolabs/zed:${ZED_SDK_MAJOR}.${ZED_SDK_MINOR}-devel-cuda${CUDA_MAJOR}.${CUDA_MINOR}-ubuntu${UBUNTU_RELEASE_YEAR}.04
# Print Ubuntu version
RUN apt-get update && apt-get install -y lsb-release

# OpenCV Version 
ARG OPENCV_VERSION=4.x

ADD ./dependencies.sh /dependencies.sh
RUN chmod +x /dependencies.sh
RUN /dependencies.sh

ADD ./opencv.sh /opencv.sh
RUN chmod +x /opencv.sh
RUN /opencv.sh

ADD ./tensorrt_inference.sh /tensorrt_inference.sh
RUN chmod +x /tensorrt_inference.sh
RUN /tensorrt_inference.sh

ADD ./ros-environment.sh /ros-environment.sh
RUN chmod +x /ros-environment.sh
RUN /ros-environment.sh

WORKDIR /

 CMD ["bash"]
