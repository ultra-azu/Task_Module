#!/usr/bin/env python

import rospy
import smach
from uuv_control_msgs.srv import GoTo
from geometry_msgs.msg import PoseStamped
from uuv_control_msgs.msg import Waypoint

class UpdatePoseState(smach.State):
    def __init__(self, group_name, target_pose, debug=False):
        smach.State.__init__(self, outcomes=['success', 'aborted', 'preempted'])
        self.group_name = group_name
        self.pose = None
        self.camera_data = None
        self.imu_data = None
        self.dvl_data = None
        self.target_pose = target_pose
        self.goto_service = rospy.ServiceProxy('/your_goto_service', GoTo)
        self.debug = False

        # Subscribe to the camera, IMU, and DVL topics
        # self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.camera_callback)
        # self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        # self.dvl_sub = rospy.Subscriber('/dvl/data', TwistStamped, self.dvl_callback)

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
        # Prepare the waypoint message
        waypoint = Waypoint()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "world"  # Change this as per your coordinate frame
        waypoint.point = self.target_pose.position
        waypoint.max_forward_speed = 1.0
        waypoint.heading_offset = 0.0
        waypoint.use_fixed_heading = False
        waypoint.radius_of_acceptance = 2.0  # Set as per requirement

        # Prepare the GoTo service request
        goto_request = GoToRequest()
        goto_request.waypoint = waypoint
        goto_request.interpolator = 'linear'

        try:
            response = self.goto_service(goto_request)
            if response.success:
                return 'success'
            else:
                return 'aborted'
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'

class UpdatePoseToObjectState(UpdatePoseState):
    def __init__(self, group_name, object_topic, desired_object_name,debug):
        super(UpdatePoseToObjectState, self).__init__(group_name,debug)
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
        if self.object_data is not None:
            # Prepare the waypoint message for the object's position
            waypoint = Waypoint()
            waypoint.header.stamp = rospy.Time.now()
            waypoint.header.frame_id = "world"
            waypoint.point = self.object_data.position  # Assuming object_data is a Pose
            # ... set other waypoint attributes ...

            # Prepare and call GoTo service
            goto_request = GoToRequest()
            goto_request.waypoint = waypoint
            # ... set other request attributes ...

            try:
                response = self.goto_service(goto_request)
                return 'success' if response.success else 'aborted'
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
                return 'aborted'
        else:
            return 'aborted'


class Rotate90DegreesState(UpdatePoseState):
    def __init__(self, group_name, debug=False):
        super(Rotate90DegreesState, self).__init__(group_name, debug)
        pass

    def execute(self, userdata):
        # Get the current pose
        current_pose = # obtain current pose

        # Compute new orientation (example: rotate 90 degrees around Z-axis)
        new_orientation = # calculate new orientation

        # Prepare the waypoint for the new orientation
        waypoint = Waypoint()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "world"
        waypoint.point = current_pose.position
        waypoint.heading_offset = # heading offset for 90-degree rotation
        # ... set other waypoint attributes ...

        # Prepare and call GoTo service
        goto_request = GoToRequest()
        goto_request.waypoint = waypoint
        # ... set other request attributes ...

        try:
            response = self.goto_service(goto_request)
            return 'success' if response.success else 'aborted'
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'
        
# TODO: FIx the Lower Depth Class
class LowerDepth(smach.State):
    def __init__(self, controls):
        smach.State.__init__(self, outcomes=['success', 'aborted', 'preempted'])
        self.controls = controls
        self.first_pose = True

    def execute(self, userdata):
        if self.first_pose:
            self.hold_pose = self.controls.get_state().pose.pose
            self.controls.move_to_pose_global(self.hold_pose)
            self.first_pose = False

        return 'done'

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
    

