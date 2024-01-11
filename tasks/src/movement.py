import rospy
import smach
from uuv_control_msgs.srv import GoTo, GoToRequest
from geometry_msgs.msg import Pose, PoseStamped, Point
from uuv_control_msgs.msg import Waypoint
from uuv_control_msgs.srv import Hold, HoldRequest

import rospy
import smach
import random
from std_msgs.msg import Time
from geometry_msgs.msg import Point
from uuv_control_msgs.srv import InitWaypointSet, InitWaypointSetRequest
from uuv_control_msgs.msg import Waypoint
from robot_math import compare_poses


class UpdatePoseState(smach.State):
    def __init__(self, edge_case_callback,next_state_callback = None , pose = None,num_waypoints=1):
        smach.State.__init__(self, outcomes=['success', 'edge_case_detected', 'aborted'],
                             input_keys=['shared_data'],
                             output_keys=['shared_data'])
        self.edge_case_callback = edge_case_callback
        self.next_state_callback = next_state_callback
        self.num_waypoints = num_waypoints
        self.waypoints = []
        self.init_waypoint_set_service = rospy.ServiceProxy('init_waypoint_set', InitWaypointSet)
        self.pose = pose


    @staticmethod
    def generate_waypoints(num_waypoints):
        waypoints = []
        for _ in range(num_waypoints):
            waypoint = Waypoint()
            waypoint.point = Point(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10))
            waypoint.max_forward_speed = random.uniform(0, 5)
            waypoint.heading_offset = random.uniform(-3.14, 3.14)
            waypoint.use_fixed_heading = random.choice([True, False])
            waypoint.radius_of_acceptance = random.uniform(0, 5)
            waypoints.append(waypoint)
        return waypoints
    

    def WaypointFromPose(self):
        waypoints = []
        waypoint = Waypoint()
        waypoint.point = self.pose.point
        waypoint.max_forward_speed = random.uniform(0, 5)
        waypoint.heading_offset = random.uniform(-3.14, 3.14)
        waypoint.use_fixed_heading = random.choice([True, False])
        waypoint.radius_of_acceptance = random.uniform(0, 5)
        waypoints.append(waypoint)
    

    def pose_reached(self, current_pose, destination_pose, threshold):
        # Check if the current pose is within a certain threshold of the destination pose
        # The function 'compare_poses' should return True if the poses are similar within the threshold
        return compare_poses(current_pose, destination_pose, threshold)

    def execute(self, userdata):
        if userdata:
            shared_data = userdata.shared_data

        # Call InitWaypointSet service
        try:
            # waypoints = self.generate_waypoints()
            if self.pose:
                waypoints = self.WaypointFromPose()
            else:
                waypoints = self.generate_waypoints(self.num_waypoints)
            req = InitWaypointSetRequest()
            req.start_time = Time()  # Zero value by default
            req.start_now = True
            req.waypoints = waypoints
            req.max_forward_speed = 1.5
            req.heading_offset = 0.0
            req.interpolator = 'linear'

            response = self.init_waypoint_set_service(req)
            if not response.success:
                rospy.logerr("Failed to initiate InitWaypointSet service.")
                return 'aborted'
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'

         # Monitoring loop
        while not rospy.is_shutdown():

            # Check if the destination has been reached
            if self.pose_reached(userdata.shared_data.current_pose, self.zedObject2Waypoint().point, threshold=YOUR_DEFINED_THRESHOLD):
                rospy.loginfo("Destination has been reached.")
                return 'success'

            if self.edge_case_callback(shared_data):
                rospy.logwarn("Edge case detected, transitioning to handle situation.")
                userdata.edge_case = self.next_state_callback()
                return "edge_case_detected"

            rospy.sleep(0.1)  # Sleep to prevent a busy loop, adjust as needed

        return 'aborted'


class UpdatePoseToObjectState(UpdatePoseState):
    def __init__(self, desired_object_name, edge_case_callback,next_state_callback ):
        super(UpdatePoseToObjectState, self).__init__(outcomes=['success', 'edge_case_detected', 'aborted', "object_not_detected"],
                                                      input_keys=['shared_data'],
                                                      output_keys=['edge_case'],
                                                 edge_case_callback=edge_case_callback,
                                                   next_state_callback=next_state_callback)
        self.desired_object_name = desired_object_name
        self.object_data = None

    def zedObject2Waypoint(self):
        object_waypoint = Waypoint()
        object_waypoint.header.stamp = rospy.Time.now()
        object_waypoint.header.frame_id = "world"
        object_waypoint.point = self.object_data.position  #
        return object_waypoint

    def execute(self, userdata):
        shared_data = userdata.shared_data
        for object in  shared_data.zed_data["ObjectsStamped"]:
            if self.desired_object_name == object.label:
                self.object_data = object

        if self.object_data:
            object_waypoint = self.zedObject2Waypoint()
            self.waypoints.append(object_waypoint)

        else:
            # Failed to identify object
            return "object_not_detected"

        # Prepare and call InitWaypointSet service
        try:
            req = InitWaypointSetRequest()
            req.start_time = Time()  # Zero value by default
            req.start_now = True
            req.waypoints = self.waypoints
            req.max_forward_speed = 1.5
            req.heading_offset = 0.0
            req.interpolator = 'linear'

            response = self.init_waypoint_set_service(req)
            if not response.success:
                rospy.logerr("Failed to initiate InitWaypointSet service.")
                return 'aborted'
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'
        

        # Monitoring loop
        while not rospy.is_shutdown():

            # Check if the destination has been reached
            if self.pose_reached(userdata.shared_data.current_pose, self.zedObject2Waypoint().point, threshold=10):
                rospy.loginfo("Destination has been reached.")
                return 'success'

            if self.edge_case_callback(shared_data):
                rospy.logwarn("Edge case detected, transitioning to handle situation.")
                userdata.edge_case = self.next_state_callback()
                return "edge_case_detected"

            rospy.sleep(0.1)  # Sleep to prevent a busy loop, adjust as needed

        return 'aborted'

# TODO: uuv simulator do not have services for rotation. We will have to implemented in the control system  or hardcode a solution directly to embedded
# TODO: Describe the Transitions 
class Rotate90DegreesState(UpdatePoseState):
    def __init__(self, group_name, debug=False):
        super(Rotate90DegreesState, self).__init__(group_name, debug)
        pass

    def execute(self, userdata):
        pass
        

class HoldPositionTask(smach.State):
    """Hold position at the place the robot is at the first time this runs"""

    def __init__(self, time_to_hold, hold_server_topic):
        """
        Parameters:
            time_to_hold (int): Duration to hold the position in seconds
            hold_server_topic (str): The ROS service topic for holding position
        """
        smach.State.__init__(self, outcomes=['success', 'aborted'])
        self.time_to_hold = rospy.Duration(time_to_hold)
        self.hold_server_topic = hold_server_topic
        self.hold_service = rospy.ServiceProxy(hold_server_topic, Hold)

    def execute(self, userdata):
        try:
            rospy.wait_for_service(self.hold_server_topic)
            response = self.hold_service()
            if not response.success:
                rospy.loginfo("Vehicle hold initiation failed.")
                return 'aborted'

            rospy.loginfo("Vehicle hold successfully initiated.")
            start_time = rospy.Time.now()
            while rospy.Time.now() - start_time < self.time_to_hold:
                if rospy.is_shutdown():
                    return 'aborted'
                rospy.sleep(0.1)

            return 'success'
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'