# Copyright (c) 2023 Real-Time Development Center (RTDC) Project
# All rights reserved.

# ROS Message that represents the most notable values of the system setup 
# available for the Teledyne Marine RD Instruments Doppler Velocity Logger (DVL)

int8 sw_trigger # Flag that enables a software trigger for the Wayfinder. 1 enables the software trigger, otherwise it will be disabled.
float32 max_depth # Maximum tracking depth in meters.
float32 max_track_range # Maximum vertical beam range in meters.