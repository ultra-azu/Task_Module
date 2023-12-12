#!/usr/bin/env python3

"""ROS Node Executable Class for Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) Sensor 
# Copyright (c) 2023 Real-Time Development Center (RTDC) Project
# All rights reserved.
#
# @file: dvl_node.py 
# @author: RTDC Capstone team (rtdc@upr.edu)
# @date: Spring 2023
#
# @brief ROS DVL Node Executable Script
#
# This executable provides a ros executable node class that includes methods for communicating and integrating the python3 driver available for the Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) Sensor 
# by using the ros python3 client library available for ROS Noetic. 
#
#
# Sensors: 
# 1x Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) Sensor
#
# Python Classes/Objects:
# WayfinderNode(NODE_NAME, TOPIC_NAME, RATE, QUEUE_SIZE)
#     - Parameter: 
#         - NODE_NAME                           : string                                                                    -> Name of Node 
#         - TOPIC_NAME                          : string                                                                    -> Name of topic 
#         - RATE                                : int                                                                       -> To maintain a specific rate for the loop
#         - QUEUE_SIZE                          : int                                                                       -> To set the size of the outgoing message queue of the ROS topic to be used for the DVL
#
#       - Private Variables
#         @priv _logger                         : FileIO                                                                    -> Keeps an additional local log file inside the /logs directory
#         @priv _log_file_counter               : int                                                                       -> Stores the current local log attempt
#         @priv _ROLL                           : np.radians                                                                -> Angle through which the AUV must be rotated about its X axis 
#         @priv _PITCH                          : np.radians                                                                -> Angle through which the AUV must be rotated about its Y axis
#         @priv _YAW                            : np.radians                                                                -> Angle through which the AUV must be rotated about its Z axis
#         @priv _prev_time                      : int                                                                       -> To store previous time for DVL#  

#     - Private Methods
#         @priv _setup_private_variables        : ()                                                                        -> None             ->  Initialize all the private variables for node.
#         @priv _setup_logger                   : ()                                                                        -> None             ->  Initialize local log file inside the logs directory for each run.
#         @priv _setup_transposition_matrix     : ()                                                                        -> None             ->  Sets up the Instrument to Ship Rotation Matrix based on the Wayfinder guide
#         @priv  _setup_ros_dependancies        : (NODE_NAME: Any, TOPIC_NAME: Any, RATE: Any, QUEUE_SIZE: Any)             -> None             ->  Sets up all the ros dependancies required
#         @priv  _setup_wayfinder               : ()                                                                        -> None             ->  Connects and configures the various system settings of the Wayfinder
#         @priv _binary_data_output_group_cb    : ( output_data: OutputData, *args: Any)                                    -> None             ->  Callback function that is executed each time a software trigger is sent
#         @priv _publish                        : ()                                                                        -> None             ->  Loop function that sends a software trigger command to Wayfinder and publishes the message
#         @priv _set_msg_header                 : ()                                                                        -> Header           ->  Sets the frame_id and ros timestamp for the dvl publisher  
#
"""

"""
# @module rospy
# 
# @classes
#   - Publisher(name, type_of_message, queue_size=None)         : ROS Communication Tool that publishes a specific type of ROS message over a given ROS topic
#   - Rate (hz, reset=False)                                    : Provides a particular rate for a node loop
#   - Time                                                      : ROS Built-in Time library.
# @functions 
#   - get_param   : (param_name: Any, default: _Unspecified = _unspecified) -> (Any | _Unspecified) - Retrieve a parameter from the param server
#   - init_node   : init_node: (name: Any, argv: Any | None = None, anonymous: bool = False) -> Any
#   - loginfo     : (msg: Any, *args: Any, **kwargs: Any)                   -> None - Logging level used by rosconsole for small amounts of information that may be useful to a user.
#   - logwarn     : (msg: Any, *args: Any, **kwargs: Any)                   -> None - Logging level used by rosconsole for information that the user may find alarming, and may affect the output of the application, but is part of the expected working of the system.
#   - logerr      : (msg: Any, *args: Any, **kwargs: Any)                   -> None - Logging level used by rosconsole for something serious (but recoverable) has gone wrong.
#   - Time.now    : (                                   )                   -> Time  - Create new Time instance representing current ros time.
#
# @brief ROS Client library for python that provides multiple methods that publish and log data from a ROS custom message of type 'DVL_MSG'
# 
# @see For more information on the ROS client library for Python visit
# http://wiki.ros.org/rospy
"""
import rospy

"""
# @module Header
# 
# @brief ROS python message used to include timestamp data on a given coordinate frame
# 
# @see For more information on the ROS Header Message for Python visit
# http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Header.html
"""
from std_msgs.msg import Header

"""
# @module DVL_MSG
# 
# @brief ROS python message used to integrate the system setup and binary output data output group of the Wayfinder sensor.
# 
# @see For more information on the ROS DVL_MSG Message visit
# nav_sensors/msg/DVL_MSG.msg
"""
from nav_sensors.msg import DVL_MSG

"""
# @module Dvl
# 
# @brief Python3 Driver for the Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) Sensor.
# Enables a communication protocol via USB RS-232 that allows the Jetson the complete control of the sensor.
# 
# @see For more information on the python3 driver for the Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL)
# https://teledynerdi.myshopify.com/pages/wayfinder-driver-index
#@see For more information on the Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL)
#https://cdn.shopify.com/s/files/1/0407/2312/0282/files/Wayfinder_DVL_Guide.pdf?v=1603393754
"""
from dvl.dvl import Dvl  # Import wayfinder dvl module from Teledyne Marine RDI

"""
# @module OutputData
#
# @brief Class from the Python3 Driver of the Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) Sensor
# that provides output ping data and will be used as the type of input for the Wayfinder callback function.
# 
# @see For more information on the OutputData Class of the Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) visit 
# https://teledynerdi.myshopify.com/pages/wayfinder-driver-system.html#dvl.system.OutputData
"""
from dvl.system import OutputData

"""
# @module io
# @class FileIO
#
# @brief Python's main facility that provides and interface for various input/output files in the container's file system.
# 
# @see For more information on the FileIO class of the io module visit
# https://docs.python.org/3/library/io.html#io.FileIO
"""
from io import FileIO  

"""
# @module Time
# @function sleep
#
# @brief Python library that provides a suspend execution of the calling thread for the given number of seconds. The argument may
# be a floating point number to indicate a more precise sleep time.
# 
# @see For more information on the sleep function of the time module 
# https://docs.python.org/3/library/io.html#io.FileIO
"""
from time import sleep

"""
# @module os
#
# @brief Python library that provides a portable way of using operating system dependent functionality.
# Used to join two or more pathname components.
# 
# @see For more information on the os module visit
# https://docs.python.org/3/library/os.html
"""
import os  

"""
# @module traceback
#
# @brief Standard Python interface to extract, format and print information about Python stack traces.
# 
# @see For more information on the traceback module visit
# https://docs.python.org/3/library/traceback.html
"""
import traceback  

"""
# @module numpy as np
# 
# @functions array radians sin cos
#
# @brief Fundamental module that provides scientific computing in Python.
# 
# @see For more information on the numpy module visit
# https://numpy.org/devdocs/user/whatisnumpy.html
"""
import numpy as np  # Used for mathematical calculations

"""
# @module math
# 
# @function isnan(x) -> Return true if x is a NaN (not a number), and False otherwise
#
# @brief Python module that provides access to the mathematical functions defined by the C standard.
# One mathematical function to be called 'isnan' for checking for NaN (not a number) velocity values. 
# 
# @see For more information on the math module visit
https://docs.python.org/3/library/math.html#math.isnan
"""
import math  # To check for Not a Number (NaN) Velocities

"""
# @module datetime as dt
# 
# @class datetime           -> Manipulates dates and times
# 
# @function 
#   datetime.now()          -> Construct a datetime from time.time() and optional time zone info.
#   timedelta(seconds=x)    -> Represent the difference between two datetime objects.
#
# @brief Python module that allows users to set a Real-Time Clock (RTC)
#
# @see For more information on the math module visit
https://docs.python.org/3/library/math.html#math.isnan
"""
import datetime as dt  # To set real-time clock

"""
Declaration of global variables for ROS Node Executable Class:
# ROS MACROS:
#     _NODE_NAME                :   str         ->
#     _TOPIC_NAME               :   str         ->
#     _TRANSMISSION_RATE        :   int         ->
#     _QUEUE_SIZE               :   int         ->
#     _FRAME_ID                 :   str         ->
# 
# WAYFINDER MACROS:
#     _WAYFINDER_SW_TRIGGER     :   int         ->
#     _WAYFINDER_BAUDRATE       :   int         ->        
#     _WAYFINDER_PORT           :   str         ->
#     _WAYFINDER_MAX_DEPTH      :   int         ->
#     _WAYFINDER_MAX_RANGE      :   int         ->
#     _WAYFINDER_PING_TIME      :   int         ->
#     _ROLL_ANGLE               :   int         ->
#     _PITCH_ANGLE              :   int         ->       
#     _START_TIME               :   int         ->
# 
# LOGGER MACROS:
#     _PREV_LOGGER_DIR          :   str         ->
#     _PREV_LOGGER_PREF         :   str         ->
#     _PREV_OS_PATH             :   str         ->
#     _LOGGER_DIR               :   str         ->
#     _LOGGER_NUM_FORMAT        :   str         ->
#     _WRITE_COMMAND            :   str         ->
#     _READ_COMMAND             :   str         ->
"""

# Retrieve the node name parameter from the param server
_NODE_NAME = rospy.get_param("/wayfinder/node_name")

#Retrieve a parameter from the param server
_TOPIC_NAME = rospy.get_param("/wayfinder/topic_name")
_TRANSMISSION_RATE = rospy.get_param("/wayfinder/transmission_rate")
_QUEUE_SIZE = rospy.get_param("/wayfinder/queue_size")
_FRAME_ID = rospy.get_param("/wayfinder/frame_id")

# MACROS (M) for WAYFINDER (DVL) Macros Declarations

_WAYFINDER_SW_TRIGGER = rospy.get_param("/wayfinder/sw_trigger")
_WAYFINDER_BAUDRATE = rospy.get_param("/wayfinder/baudrate")
_WAYFINDER_PORT = rospy.get_param("/wayfinder/port")
_WAYFINDER_MAX_DEPTH = rospy.get_param("/wayfinder/max_depth")
_WAYFINDER_MAX_RANGE = rospy.get_param("/wayfinder/max_range")
_WAYFINDER_PING_TIME = rospy.get_param(
    "/wayfinder/ping_time")  # second ping time

_ROLL_ANGLE = rospy.get_param("/wayfinder/roll_angle")
_PITCH_ANGLE = rospy.get_param("/wayfinder/pitch_angle")
_YAW_ANGLE = rospy.get_param("/wayfinder/yaw_angle")

_START_TIME = rospy.get_param("/wayfinder/start_time")

# MACROS for Path to be used to determine the name of the logger file to be created based on the current
# log files inside the directory
_PREV_LOGGER_DIR = rospy.get_param("/wayfinder/prev_logger_dir")
_PREV_LOGGER_PREF = rospy.get_param("/wayfinder/prev_logger_pref")
_PREV_OS_PATH = os.path.join(_PREV_LOGGER_DIR, _PREV_LOGGER_PREF)

# MACROS for Path to be used as the log file for the dvl
_LOGGER_DIR = rospy.get_param("/wayfinder/logger_dir")
_LOGGER_NUM_FORMAT = rospy.get_param("/wayfinder/logger_num_format")
_WRITE_COMMAND = rospy.get_param("/wayfinder/write_command")
_READ_COMMAND = rospy.get_param("/wayfinder/read_command")




class WayfinderNode():
    """Help on ROS Node Executable Class for Teledyne's Marine RDI Wayfinder Doppler Velocity Logger (DVL) 
        NAME 
             @class WayfinderNode
    
        DESCRIPTION:
             @brief The WayfinderNode class mainly utilizes the python3 driver provided by Wayfinder and various ros communication tools from the ros python client library
             to interface and map the values from the system setup and binary data output group of the Wayfinder sensor to a custom ros message of type 'DVL_MSG' for publishing. 
             The system setup information features multiple calibration parameters that effect the perfomance of the sensor such as the software trigger flag (0-disabled, 1-enabled), 
             the maximum depth and the maximum tracking bottom range (in meters). On the other hand, the binary data output group is mostly mapped to custom ros messages for data and power. 
             The data information features the wayfinder instrument (X, Y, Z) coordinates (velocity measurements in m/s), velocity error (in m/s), beams (4 beams) (in meters), 
             mean_bottom_range (in meters) and speed_of_sound (m/s). The final part of the ros message is extracted from the power information of the binary data output group which features 
             the input voltage (in volts), transmit voltage (in volts) and current (in amps) information of the Wayfinder.
    
        USAGE:
             @example-1 roslaunch nav_sensors hydrus_launch
             @example-2 (If roscore is running and config file is loaded to rosparam)
                       rosrun nav_sensors hydrus_launch
        PARAMS:
            @param NODE_NAME: Name of ROS Node to be initilized
            @param TOPIC_NAME: Name of ROS Topic to be published
            @param RATE: Hz rate to determine how much ros node will sleep
            @param QUEUE_SIZE: Size used to publish message from different threads asynchronously
    
        PACKAGE CONTENTS
          - Private Variables
            @priv _logger                         : FileIO                                                                    -> Keeps an additional local log file inside the /logs directory
            @priv _log_file_counter               : int                                                                       -> Stores the current local log attempt
            @priv _ROLL                           : np.radians                                                                -> Angle through which the AUV must be rotated about its X axis 
            @priv _PITCH                          : np.radians                                                                -> Angle through which the AUV must be rotated about its Y axis
            @priv _YAW                            : np.radians                                                                -> Angle through which the AUV must be rotated about its Z axis
     
        - Private Methods
            @priv _setup_private_variables        : ()                                                                        -> None             ->  Initialize all the private variables for node.
            @priv _setup_logger                   : ()                                                                        -> None             ->  Initialize local log file inside the logs directory for each run.
            @priv _setup_transposition_matrix     : ()                                                                        -> None             ->  Sets up the Instrument to Ship Rotation Matrix based on the Wayfinder guide
            @priv  _setup_ros_dependancies        : (NODE_NAME: Any, TOPIC_NAME: Any, RATE: Any, QUEUE_SIZE: Any)             -> None             ->  Sets up all the ros dependancies required
            @priv  _setup_wayfinder               : ()                                                                        -> None             ->  Connects and configures the various system settings of the Wayfinder
            @priv _binary_data_output_group_cb    : ( output_data: OutputData, *args: Any)                                    -> None             ->  Callback function that is executed each time a software trigger is sent
            @priv _publish                        : ()                                                                        -> None             ->  Loop function that sends a software trigger command to Wayfinder and publishes the message
            @priv _set_msg_header                 : ()                                                                        -> Header           ->  Sets the frame_id and ros timestamp for the dvl publisher  
    
        RESOURCES
           @see For more information on the ROS client library for Python visit
           http://wiki.ros.org/rospy
           @see For more information on the python3 driver for the Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL)
           https://teledynerdi.myshopify.com/pages/wayfinder-driver-index
           @see For more information on the Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL)
           https://cdn.shopify.com/s/files/1/0407/2312/0282/files/Wayfinder_DVL_Guide.pdf?v=1603393754
    """

    def __init__(self, NODE_NAME, TOPIC_NAME, RATE, QUEUE_SIZE):
        """ Help on initialize method for ROS Wayfinde Node
                NAME 
                     @method __init__
            
                DESCRIPTION:
                     @brief This python method initializes, sets up and runs the executable class along with all of its dependancies
            .
                PARAMS:
                    @param NODE_NAME: Name of ROS Node to be initilized
                    @param TOPIC_NAME: Name of ROS Topic to be published
                    @param RATE: Hz rate to determine how much ros node will sleep
            
               Methods:
                    @priv _setup_private_variables        : ()                                                                        -> None             ->  Initialize all the private variables for node.
                    @priv _setup_logger                   : ()                                                                        -> None             ->  Initialize local log file inside the logs directory for each run.
                    @priv _setup_transposition_matrix     : ()                                                                        -> None             ->  Sets up the Instrument to Ship Rotation Matrix based on the Wayfinder guide
                    @priv  _setup_ros_dependancies        : (NODE_NAME: Any, TOPIC_NAME: Any, RATE: Any, QUEUE_SIZE: Any)             -> None             ->  Sets up all the ros dependancies required
                    @priv  _setup_wayfinder               : ()                                                                        -> None             ->  Connects and configures the various system settings of the Wayfinder
                    @priv _publish                        : ()                                                                        -> None             ->  Loop function that sends a software trigger command to Wayfinder and publishes the message
        """
        self._setup_private_variables()
        self._setup_logger()
        self._setup_transposition_matrix()
        self._setup_ros_dependancies(NODE_NAME, TOPIC_NAME, RATE, QUEUE_SIZE)
        self._setup_wayfinder()
        self._publish()

    def _setup_private_variables(self):
        """ Help on setup private variables method for ROS Wayfinde Node
                NAME 
                     @method _setup_private_variables
            
                DESCRIPTION:
                     @brief This python method sets up multiple inner private variables
        
                VARIABLES
                     @priv _logger                         : FileIO                                                                    -> Keeps an additional local log file inside the /logs directory
                     @priv _log_file_counter               : int                                                                       -> Stores the current local log attempt
                     @priv _ROLL                           : np.radians                                                                -> Angle through which the AUV must be rotated about its X axis 
                     @priv _PITCH                          : np.radians                                                                -> Angle through which the AUV must be rotated about its Y axis
                     @priv _YAW                            : np.radians                                                                -> Angle through which the AUV must be rotated about its Z axis
                     @priv _prev_time                      : int                                                                       -> To store previous time for DVL

        """
        self._logger: FileIO
        self._log_file_counter: int
        self._ROLL: np.radians = np.radians(_ROLL_ANGLE)
        self._PITCH: np.radians = np.radians(_PITCH_ANGLE) 
        self._YAW: np.radians = np.radians(_YAW_ANGLE)  
        self._prev_time: int = _START_TIME

    def _setup_logger(self):
        """ Help on setup logger method for ROS Wayfinde Node
                NAME 
                     @method _setup_logger
            
                DESCRIPTION:
                     @brief This python method sets up a local log file each run in the logs directory of the nav_sensors ros pkg
        """
        try:
            # open prev log file to view the current log_num
            with open(_PREV_OS_PATH, _READ_COMMAND) as prev_log:
                # extract log_num
                self._log_file_counter = int(prev_log.readline()) + 1
        except Exception: # handle the case that there is no number on the log file
            # 0 will be assigned if there is not a number
            self._log_file_counter = 0

        # Open attempts file to update log_num
        with open(_PREV_OS_PATH, _WRITE_COMMAND) as lastLog:
            lastLog.write("%d\n" % self._log_file_counter)

        # Create local logger /nav_sensors/logs
        self._logger = open(os.path.join(
            _LOGGER_DIR, _LOGGER_NUM_FORMAT % self._log_file_counter), _WRITE_COMMAND)

    def _setup_transposition_matrix(self):
        """ Help on setup transposition method for ROS Wayfinde Node
            NAME 
                 @method _setup_transposition_matrix
        
            DESCRIPTION:
                 @brief This python method provides the Instrument to Ship rotation matrix according to Wayfinder Manual
        """
        self._transposition_matrix = np.array(
            [
                [np.cos(self._YAW) * np.cos(self._PITCH),
                 np.cos(self._YAW) * np.sin(self._PITCH) *
                    np.sin(self._ROLL) - np.sin(self._YAW) *
                 np.cos(self._ROLL),
                 np.cos(self._YAW) * np.sin(self._PITCH) *
                    np.cos(self._ROLL) + np.sin(self._YAW) * np.sin(self._ROLL)
                 ],
                [np.sin(self._YAW) * np.cos(self._PITCH),
                 np.sin(self._YAW) * np.sin(self._PITCH) *
                    np.sin(self._ROLL) + np.cos(self._YAW) *
                 np.cos(self._ROLL),
                 np.sin(self._YAW) * np.sin(self._PITCH) *
                    np.cos(self._ROLL) - np.cos(self._YAW) * np.sin(self._ROLL)
                 ],
                [-np.sin(self._PITCH),
                 np.cos(self._PITCH) * np.sin(self._ROLL),
                 np.cos(self._PITCH) * np.cos(self._ROLL)
                 ]
            ])

    def _setup_ros_dependancies(self, NODE_NAME, TOPIC_NAME, RATE, QUEUE_SIZE):
        """ Help on setup ros dependacies method for ROS Wayfinde Node
            NAME 
                 @method _setup_ros_dependancies
        
            DESCRIPTION:
                 @brief This python method sets up multiple ros dependancies for the Wayfinder node such as the custom ROS message of type 
                 DVL_MSG and the Publisher for the DVL sensor. The method also initializes the node and sets up a rate for the publisher.
        
            PARAMS:
                @param NODE_NAME: Name of ROS Node to be initilized
                @param TOPIC_NAME: Name of ROS Topic to be published
                @param RATE: Hz rate to determine how much ros node will sleep
                @param QUEUE_SIZE: Size used to publish message from different threads asynchronously
        """
        # Create a ROS message instance of type DVL_MSG to store DVL messages of Wayfinder DVL Information
        self.dvl_msg = DVL_MSG()
        
        # Instance of ROS Publisher to publish DVL information
        self.pub = rospy.Publisher(TOPIC_NAME, DVL_MSG, queue_size=QUEUE_SIZE)
        
        # Initialize ROS Node
        rospy.init_node(NODE_NAME, anonymous=True)  
        
        # Setup rate for publisher
        self.rate = rospy.Rate(RATE)

    def _setup_wayfinder(self):
        """ Help on setup wayfinder method for ROS Wayfinde Node
            NAME 
                 @method _setup_wayfinder
        
            DESCRIPTION:
                 @brief This python method provides:
                      - A connection to the Wayfinder sensor at a given port and baudrate, 
                      - Once connected, the wayfinder will be reset to default factory settings, 
                      - The system setup will be calibrated to satisfy user environment conditions 
                      - It will register a callback method to be executed,
                       - It will enter command mode to set the real-time clock of the Wayfinder
        
        """
        # Create instance of Teledyne Marine RDInstruments Doppler Velocity Logger from Wayfinder Driver Library
        self.wayfinder = Dvl()
        msg = self.dvl_msg

        # Use the connect method of the wayfinder to attempt a connection with the sensor based on the given port and baudrate:
        # If succedeed, the loggers will continue with the algorithm and log the file locally and via rosconsole,
        # otherwise, the loggers will log a connection error message until the sensor is connected.
        while not self.wayfinder.connect(_WAYFINDER_PORT, _WAYFINDER_BAUDRATE):
            # Log a connection error locally
            rospy.logerr(
                "Error connecting to Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) sensor")
            self._logger.write(
                "Error connecting to Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) sensor \n")
        
        # Otherwise, log that the connection was succesful
        rospy.loginfo(
            "Succesfully Connected to Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) sensor")
        self._logger.write(
            "Succesfully Connected to Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) sensor \n")

        if self.wayfinder.reset_to_defaults():  # reset sensor to default factory settings
            self._logger.write("DVL is reset to default factory settings \n")
        else:
            # Log an error message
            rospy.logerr(
                "Failed to reset Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) sensor to default factory settings ")
            self._logger.write(
                "Failed to reset Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) sensor to default factory settings \n")

        # Use the get_setup() method of the Wayfinder wrapper to get user system setup and calibrate
        # the sensor to any oceanic environment.
        if not self.wayfinder.get_setup():  # If the wayfinder cannot get the setup of the system,
            rospy.logerr(
                "Failed to get system setup of the Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) sensor")
            # log a System Setup Error
            self._logger.write(
                "Failed to get system setup of Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) sensor \n")
        else:  # otherwise

            # Modify system setup structure:

            SETUP = self.wayfinder.system_setup  # declare the object for the sensor

            rospy.loginfo(
                "System setup of Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) sensor before calibration: %s " % (SETUP))

            # log the system setup before calibration
            self._logger.write(
                'System setup of Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) sensor before calibration: %s \n' % SETUP)

            # Set software trigger flag
            SETUP.software_trigger = _WAYFINDER_SW_TRIGGER
            # Set maximum tracking depth in meters
            SETUP.max_depth = _WAYFINDER_MAX_DEPTH
            # Set maximum vertical beam range in meters
            SETUP.max_vb_range = _WAYFINDER_MAX_RANGE

            # extract software trigger information (0-disabled, 1-enabled)
            msg.setup.sw_trigger = SETUP.software_trigger
            # extract maximum tracking depth information (in meters)
            msg.setup.max_depth = SETUP.max_depth
            # extract maximum vertical beam range information (in meters)
            msg.setup.max_track_range = SETUP.max_vb_range

            rospy.loginfo(
                "System setup of Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) after calibration: %s " % (SETUP))

            # log system setup after calibration
            self._logger.write(
                'System setup of Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) after calibration: %s \n' % SETUP)

            # Setup new calibration settings
            # if the setup cannot be set,
            if not self.wayfinder.set_setup(SETUP):
                # log a system setup error to rosconsole
                rospy.loginfo(
                    "Failed system setup of Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL)")
                # log a system setup error to log file
                self._logger.write(
                    "Failed system setup of Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) \n")
            else:  # otherwise

                # Log information
                rospy.loginfo(
                    "Registering ondata callback function for Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL)")
                
                # register callback function to be exectuted
                self.wayfinder.register_ondata_callback(
                    self._binary_data_output_group_cb, None)
                
                # check if the system has not exited command mode
                if not self.wayfinder.exit_command_mode():
                    rospy.logerr(
                        "Failed to exit the command mode of Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) ")
                    self._logger.write(
                        "Failed to exit the command mode of Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) \n")
                #log information
                rospy.loginfo(
                    'Entering the command mode of the Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL)')
                self._logger.write(
                    "Entering the command mode of the Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) \n")
                # calculate delayed to be determined (tbd)
                delay_tbd = dt.datetime.now() + dt.timedelta(seconds=_WAYFINDER_PING_TIME)
                # declare target time
                target_time = dt.datetime(delay_tbd.year, delay_tbd.month, delay_tbd.day,
                                          delay_tbd.hour, delay_tbd.minute, delay_tbd.second)
                current_time = dt.datetime.now()
                delta_time = target_time - current_time
                rospy.loginfo('Delta Time: %s' % (delta_time))
                
                # sleep for delta time
                sleep((delta_time).total_seconds())

                # Set time of Teledyne Wayfinder DVL to Real-Time Clock (RTC)
                self.wayfinder.set_time(dt.datetime.now())  # set time

    def _binary_data_output_group_cb(self, output_data: OutputData):
        """ Help on binary data output group callback method for ROS Wayfinde Node
            NAME 
                 @method _binary_data_output_group_cb
        
            DESCRIPTION:
                 @brief Callback function to be called each ping to extract the Binary Data Output Group of the DVL
                 This python method:
                      - Gets the ping time in microseconds
                      - Gets the ping time timestamp
                      - Checks for NaN and Valid Velocities and Velocity error measurements to map them into the custom ROS DVL_MSG type
                      - Checks for Valid range to bottom and mean bottom range measurements to map them into the custom ROS DVL_MSG type
                      - Extracts speed of sound  into the custom ROS DVL_MSG type
                      - Extracts power information (input voltage, transmit voltage, current)  into the custom ROS DVL_MSG type
                      - Extracts serial_number information  into the custom ROS DVL_MSG type
            PARAMS:
                @param outputdata: Provides output ping data and will be used as the type of input for the Wayfinder callback function.
        """
        try: 
            msg = self.dvl_msg

            # Get time in microseconds
            time_stamp_us = output_data.get_date_time().timestamp() * 1e6 
            self._logger.write("%9d: \n" % time_stamp_us)
            print("Time stamp us: ", time_stamp_us)

            dataTimestamp_int_us = int(time_stamp_us)
            
            #log information
            rospy.loginfo("%9d, " % dataTimestamp_int_us)
            self._logger.write("%9d, " % dataTimestamp_int_us)

            # Get timestamp
            time = output_data.get_date_time()
            txt = time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            
            # log information
            self._logger.write("DVL TimeStamp {0}\n".format(txt))
            self._logger.write("DVL TimeStamp {0}\n".format(txt))

            # Conditional statement that checks for Not A Number (NaN) velocities inside the Teledyne Wayfinder DVL
            if math.isnan(output_data.vel_x) or math.isnan(output_data.vel_y) or \
                    math.isnan(output_data.vel_z) or math.isnan(output_data.vel_err): # if the are NaN, they will be rejected
                rospy.logwarn(
                    'The Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) -XYZ velocity measurements are Not a Number (NaN) ')
                self._logger.write(
                    "The Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) -XYZ velocity measurements are Not a Number (NaN) '\n")
                return

            # Conditional statement that checks for valid velocities inside the Teledyne Wayfinder DVL
            if output_data.is_velocity_valid():  # If the velocities are valid
                # Extract Wayfinder DVL XYZ velocity measurements and store it using an np.array
                velocity = np.array(
                    [output_data.vel_x, output_data.vel_y, output_data.vel_z])
                # Assign Teledyne Wayfinder DVL velocity measurements to custom ROS DVL message
                # assign velocity-x measurement
                msg.data.velocity.x = velocity[0]
                # assign velocity-y measurement
                msg.data.velocity.y = velocity[1]
                # assign velocity-z measurement
                msg.data.velocity.z = velocity[2]
                # Log velocity values 
                rospy.loginfo('Wayfinder Velocities After Rotation: %9.3f %9.3f %9.3f' % ((velocity[0], velocity[1], velocity[2])))
                self._logger.write("Wayfinder Velocities Before Rotation: %9.3f %9.3f %9.3f, \n" %
                                   (velocity[0], velocity[1], velocity[2]))
                # Matrix multiplication for instrument to ship rotation
                velocity = np.matmul(
                    self._transposition_matrix, velocity)
                # Log velocity measurements
                rospy.loginfo('Wayfinder Velocities After Rotation: %9.3f %9.3f %9.3f' % ((velocity[0], velocity[1], velocity[2])))
                self._logger.write("%9.3f %9.3f %9.3f, \n" %
                                (velocity[0], velocity[1], velocity[2]))
                # Assign Teledyne Wayfinder DVL velocity error to custom ROS DVL message
                msg.data.vel_error = output_data.vel_err  # extract velocity error
            
            else:  # Otherwise
                # Log 'Invalid Velocity' message to rosconsle
                rospy.logwarn(
                    'The Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) -XYZ velocity measurements are invalid ')
                # Log 'Invalid Velocities' message locally
                self._logger.write(
                    'The Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) -XYZ velocity measurements are invalid \n')
                return
            
            # Conditional statement that checks if the range to bottom is valid inside the Teledyne Wayfinder DVL
            if output_data.is_range_valid():  # If the range to bottom is valid:

                # Extract Wayfinder DVL 3-Beam solution measurements and store it using an np.array
                beams = np.array([output_data.range_beam1, output_data.range_beam2,
                                 output_data.range_beam3, output_data.range_beam4])

                # Assign Teledyne Wayfinder DVL 3-Beam solution measurements to custom ROS DVL message
                msg.data.beams.beam1 = beams[0]  # extract beam1 information
                msg.data.beams.beam2 = beams[1]  # extract beam2 information
                msg.data.beams.beam3 = beams[2]  # extract beam3 information
                msg.data.beams.beam4 = beams[3]  # extract beam4 information

                # Log Teledyne Wayfinder DVL 3-Beam solution measurements locally
                rospy.loginfo("Beams: %9.3f %9.3f %9.3f %9.3f" % (
                    beams[0], beams[1], beams[2], beams[3]))
                self._logger.write("%9.3f %9.3f %9.3f %9.3f, \n" % (
                    beams[0], beams[1], beams[2], beams[3]))

                # Extract Teledyne Wayfinder DVL mean bottom range measurements to custom ROS DVL message
                msg.data.mean_bottom_range = output_data.mean_range

                # Log Teledyne Wayfinder DVL mean bottom range measurements locally
                self._logger.write("%9.3f, \n" % (output_data.mean_range))

            else:  # Otherwise
                # Log 'Invalid Range to Bottom Measurements' message to rosconsle
                rospy.logwarn(
                    "Invalid Range to Bottom Measurements for Wayfinder")
                # Log 'Invalid Range to Bottom Measurements' message locally
                self._logger.write("Invalid Range to Bottom Measurements \n")
                return

            # extract speed of sound information from Wayfinder DVL to ROS DVL MSG
            msg.data.speed_of_sound = output_data.speed_of_sound

            rospy.loginfo(
                'Extracting Power Information from Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL)')
            
            # extract input voltage information from Wayfinder DVL to ROS DVL MSG
            msg.power.input_voltage = output_data.voltage
            # extract transmit voltage information from Wayfinder DVL to ROS DVL MSG
            msg.power.transmit_voltage = output_data.transmit_voltage
            # extract transmit current information from Wayfinder DVL to ROS DVL MSG
            msg.power.transmit_current = output_data.current
            rospy.loginfo(
                'Succesfully extracted Power Information from Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL)')
            # extract serial number from Wayfinder DVL to ROS DVL MSG
            msg.serial_number = output_data.serial_number
            # assign previous time
            self._prev_time = dataTimestamp_int_us  
            # log a finalization message
            rospy.loginfo(
                'END OF MEASUREMENT')
            self._logger.write(
                'END OF MEASUREMENT\n')  # end of data measurement
        # check for exceptions
        except Exception as e:
            # log exception 
            rospy.logerr('EXCEPTION OCURRED %s %s \n' %(e, traceback.print_exc()))
            self._logger.write('EXCEPTION OCURRED %s \n' %(e,traceback.print_exc()))
        
        # flush the internal buffer of the log
        self._logger.flush() 

    def _publish(self):
        """ Help on setup wayfinder method for ROS Wayfinde Node
            NAME 
                 @method _setup_wayfinder
        
            DESCRIPTION:
                 @brief This python method will send a software trigger command and publish the information of type DVL_MSG given the user ping time until 
                 the node is shutdown or interrupted.
        """
        while not rospy.is_shutdown():
            # delay execution of the dvl ping by user ping time
            sleep(_WAYFINDER_PING_TIME)
            # execute a software trigger that will make the Wayfinder ping
            if not self.wayfinder.send_software_trigger():
                rospy.logwarn(
                    "Failed to send software trigger to Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL)")
                # log software trigger error
                self._logger.write(
                    "Failed to send software trigger to Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) \n")
            self.dvl_msg.header = self._set_msg_header()  # Sets message header to message
            rospy.loginfo(
                'Publishing Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) Message')
            self.pub.publish(self.dvl_msg)
            rospy.loginfo(
                'Logging Teledyne Marine RDI Wayfinder Doppler Velocity Logger (DVL) Message: %s' % (self.dvl_msg))
            self.rate.sleep()  # Set publishing rate for DVL node

    def _set_msg_header(self):
        """ Help on set message header method for ROS Wayfinde Node
            NAME 
                 @method _set_msg_header
        
            DESCRIPTION:
                 @brief Set the ros timestamp and frame id for the ROS wrapper
        """
        # Create ROS Header Message
        msg_header = Header()
        # Assign current ROS time to ROS Header timestamp
        msg_header.stamp = rospy.Time.now()
        # Assgin Frame ID to Header message
        msg_header.frame_id = _FRAME_ID
        return msg_header


if __name__ == '__main__':
    try:
        #Create ROS Executable Python3 Node
        WayfinderNode(_NODE_NAME, _TOPIC_NAME, _TRANSMISSION_RATE, _QUEUE_SIZE)
    # check for for operations that interrupted, e.g. due to shutdown
    except rospy.ROSInterruptException:   
        pass
