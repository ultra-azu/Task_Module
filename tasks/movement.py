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
from tf.transformations import quaternion_from_euler


class UpdatePoseState(smach.State):
    def __init__(self, group_name, target_pose):
        smach.State.__init__(self, outcomes=['success', 'aborted', 'preempted'])
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


class UpdatePoseToObjectState(UpdatePoseState):
    def __init__(self, group_name, object_topic, desired_object_name):
        super(UpdatePoseToObjectState, self).__init__(group_name)
        self.object_data = None
        self.object_topic = object_topic
        self.desired_object_name = desired_object_name

        # Subscribe to the object topic
        self.object_sub = rospy.Subscriber(self.object_topic, self.object_callback)

    def object_callback(self, data):
        for object in data.objects:
            if data.name == self.desired_object_name:
                self.object_data = data.pose

    def execute(self, userdata):
        move_group = MoveGroupCommander(self.group_name)

        if self.object_data is not None:
            # Set the target pose for the end effector to the object pose
            self.target_pose = self.object_data

            # Plan and execute the movement
            move_group.set_pose_target(self.target_pose)
            plan = move_group.go(wait=True)
            
            if plan:
                return 'success'
            else:
                return 'aborted'
        else:
            if self.camera_data is None and self.imu_data is None and self.dvl_data is None:
                # Process sensor data
                return 'preempted'
            else:
                return 'aborted'


class Rotate90DegreesState(UpdatePoseState):
    def __init__(self, group_name):
        super(Rotate90DegreesState, self).__init__(group_name)

    def execute(self, userdata):
        move_group = MoveGroupCommander(self.group_name)

        # Get the current pose
        current_pose = move_group.get_current_pose().pose

        # Compute the new orientation after a 90-degree rotation around the Z-axis
        new_orientation = quaternion_from_euler(0, 0, current_pose.orientation.z + 1.5708)  # 1.5708 radians = 90 degrees

        # Set the target pose for the end effector
        target_pose = PoseStamped()
        target_pose.header.frame_id = move_group.get_planning_frame()
        target_pose.pose.position = current_pose.position
        target_pose.pose.orientation = Quaternion(*new_orientation)

        # Plan and execute the movement
        move_group.set_pose_target(target_pose)
        plan = move_group.go(wait=True)
        
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
    

