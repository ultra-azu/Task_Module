#!/usr/bin/env python

import rospy
import smach
import smach_ros
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped


import rospy
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge, CvBridgeError


class UpdatePoseState(smach.State):
    def __init__(self, group_name, target_pose):
        smach.State.__init__(self, outcomes=['success', 'aborted'])
        self.group_name = group_name
        self.pose = None
        self.camera_data = None
        self.imu_data = None
        self.dvl_data = None
        self.cv_bridge = CvBridge()
        self.target_pose = target_pose

        # Subscribe to the camera, IMU, and DVL topics
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        self.dvl_sub = rospy.Subscriber('/dvl/data', TwistStamped, self.dvl_callback)

    def camera_callback(self, data):
        try:
            self.camera_data = self.cv_bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def imu_callback(self, data):
        self.imu_data = data

    def dvl_callback(self, data):
        self.dvl_data = data

    def execute(self, userdata):
        move_group = MoveGroupCommander(self.group_name)

        # Set the target pose for the end effector
        self.target_pose.header.frame_id = move_group.get_planning_frame()

        # Plan and execute the movement
        move_group.set_pose_target(self.target_pose)
        plan = move_group.go(wait=True)
        
        if self.camera_data is None and self.imu_data is None and self.dvl_data is None:
            # Process sensor data
            return 'preempted'
        if plan:
            return 'success'
        else:
            return 'aborted'



class HoldPositionTask(smach.State):
    """Hold position at the place the robot is at the first time this runs"""

    def __init__(self, controls):
        """
        Parameters:
            controls (interface.ControlsInterface): interface to interact with controls
        """
        super(HoldPositionTask, self).__init__(outcomes=['success','preempted', 'aborted' ])
        self.controls = controls
        self.first_pose = True

    def execute(self, userdata):
        if self.first_pose:
            self.hold_pose = self.controls.get_state().pose.pose
            self.controls.move_to_pose_global(self.hold_pose)
            self.first_pose = False

        return 'done'