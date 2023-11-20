import rospy
import smach
from uuv_control_msgs.srv import GoTo, GoToRequest
from geometry_msgs.msg import PoseStamped
from uuv_control_msgs.msg import Waypoint
from uuv_control_msgs.srv import Hold, HoldRequest

import rospy
import smach
import random
from std_msgs.msg import Time
from geometry_msgs.msg import Point
from uuv_control_msgs.srv import InitWaypointSet, InitWaypointSetRequest
from uuv_control_msgs.msg import Waypoint

class UpdatePoseState(smach.State):
    def __init__(self, edge_case_callback,next_state_callback ,num_waypoints=3):
        smach.State.__init__(self, outcomes=['success', 'edge_case_detected', 'aborted', 'preempted'],
                             input_keys=['shared_data'],
                             output_keys=['shared_data'])
        self.edge_case_callback = edge_case_callback
        self.next_state_callback = next_state_callback
        self.num_waypoints = num_waypoints
        self.init_waypoint_set_service = rospy.ServiceProxy('init_waypoint_set', InitWaypointSet)

    def generate_waypoints(self):
        waypoints = []
        for _ in range(self.num_waypoints):
            waypoint = Waypoint()
            waypoint.point = Point(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-10, 10))
            waypoint.max_forward_speed = random.uniform(0, 5)
            waypoint.heading_offset = random.uniform(-3.14, 3.14)
            waypoint.use_fixed_heading = random.choice([True, False])
            waypoint.radius_of_acceptance = random.uniform(0, 5)
            waypoints.append(waypoint)
        return waypoints

    def execute(self, userdata):
        shared_data = userdata.shared_data

        # Call InitWaypointSet service
        try:
            waypoints = self.generate_waypoints()
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
            if self.edge_case_callback(shared_data):
                rospy.logwarn("Edge case detected, transitioning to handle situation.")
                return self.next_state_callback(shared_data)

            rospy.sleep(0.1)  # Sleep to prevent a busy loop, adjust as needed

        return 'aborted'


class UpdatePoseToObjectState(UpdatePoseState):
    def __init__(self, desired_object_name, num_waypoints=3):
        super(UpdatePoseToObjectState, self).__init__(num_waypoints=num_waypoints)
        self.desired_object_name = desired_object_name

    def execute(self, userdata):
        shared_data = userdata.shared_data

        if self.desired_object_name in shared_data:
            self.object_data = shared_data[self.desired_object_name]

            # Prepare the waypoint message for the object's position
            waypoints = self.generate_waypoints()
            object_waypoint = Waypoint()
            object_waypoint.header.stamp = rospy.Time.now()
            object_waypoint.header.frame_id = "world"
            object_waypoint.point = self.object_data.position  # Assuming object_data has a 'position' attribute of type Point
            waypoints.append(object_waypoint)

            # Prepare and call InitWaypointSet service
            try:
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
        else:
            rospy.logwarn(f"Desired object '{self.desired_object_name}' not found in shared data.")
            return 'aborted'

        # Monitoring loop
        while not rospy.is_shutdown():
            if self.edge_case_callback(shared_data):
                rospy.logwarn("Edge case detected, transitioning to handle situation.")
                return self.next_state_callback(shared_data)

            rospy.sleep(0.1)  # Sleep to prevent a busy loop, adjust as needed

        return 'aborted'

# TODO: uuv simulator do not have services for rotation. We will have to implemented in the control system  or hardcode a solution directly to embedded
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