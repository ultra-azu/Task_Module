cmake_minimum_required(VERSION 2.8.3)
project(combined_package)

find_package(catkin REQUIRED COMPONENTS
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  sensor_msgs
  stereo_msgs
  dynamic_reconfigure
  nodelet
  diagnostic_updater
  geometry_msgs
  visualization_msgs
  zed_interfaces
  rospy
  smach
  smach_ros
  actionlib
  actionlib_msgs
  message_generation
  rosbag
  std_msgs
  topic_tools
  uuv_control_msgs
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  actionlib_msgs
  uuv_control_msgs
)



#############
## Install ##
#############



## mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE
)
