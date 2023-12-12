# Copyright (c) 2023 Real-Time Development Center (RTDC) Project
# All rights reserved.

# ROS Message that represents the data available for the Teledyne Marine RD Instruments
# Doppler Velocity Logger (DVL)

geometry_msgs/Vector3 velocity # Beam 1,2,3 or velocity in m/s of the DVL relative to the bottom X,Y,Z and respectively
float32 vel_error # Beam 4 or error velocity in m/s.
nav_sensors/DVL_Beams beams # Range in meters from the DVL to the bottom as determined by each Beam 1-4 (in meters)
float32 mean_bottom_range # Calculated mean range (in meters) to bottom based on valid beams
float32 speed_of_sound # The speed of sound (in m/s) used for the doppler processing