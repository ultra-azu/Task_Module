# Copyright (c) 2023 Real-Time Development Center (RTDC) Project
# All rights reserved.

# ROS Message that represents the most notable values of the binary data output group 
# available for the Teledyne Marine RDInstruments Doppler Velocity Logger (DVL)

# @module Header
# 
# @brief ROS message that provides timestamp data on a given coordinate frame
# 
# @see For more information on the ROS Header message visit
# http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html

std_msgs/Header header # Stores timestamp and frame_id values

nav_sensors/DVL_Info setup # Stores notable system information of the DVL


nav_sensors/DVL_Data data # Stores data informatin of the DVL


nav_sensors/DVL_Power power # Stores power informatin of the DVL

# serial number of the system
string serial_number 