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
import math
import tf


class UpdatePoseState(smach.State):
    def __init__(self, edge_case_callback,next_state_callback = None , pose = None,num_waypoints=1):
        smach.State.__init__(self, outcomes=['success', 'edge_case_detected', 'aborted'],
                             input_keys=['shared_data'],
                             output_keys=['shared_data'])
        self.edge_case_callback = edge_case_callback
        self.next_state_callback = next_state_callback
        self.num_waypoints = num_waypoints
        self.init_waypoint_set_service = rospy.ServiceProxy('init_waypoint_set', InitWaypointSet)
        self.pose = pose
        self.threshold = 0.05


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
    
    @staticmethod
    def WaypointFromPose(self):
        waypoints = []
        waypoint = Waypoint()
        waypoint.point = self.pose.point
        waypoint.max_forward_speed = random.uniform(0, 5)
        waypoint.heading_offset = random.uniform(-3.14, 3.14)
        waypoint.use_fixed_heading = random.choice([True, False])
        waypoint.radius_of_acceptance = random.uniform(0, 5)
        waypoints.append(waypoint)
        return waypoints
    
    @staticmethod
    def pose_reached(self, current_pose, destination_pose, threshold):
        # Check if the current pose is within a certain threshold of the destination pose
        # The function 'compare_poses' should return True if the poses are similar within the threshold
         # Calculate position difference
        position_diff = math.sqrt(
                (current_pose.position.x - destination_pose.position.x) ** 2 +
                (current_pose.position.y - destination_pose.position.y) ** 2 +
                (current_pose.position.z - destination_pose.position.z) ** 2
            )

            # Calculate orientation difference (simple method, more complex calculations may involve quaternions)
        orientation_diff = math.sqrt(
                (current_pose.orientation.x - destination_pose.orientation.x) ** 2 +
                (current_pose.orientation.y - destination_pose.orientation.y) ** 2 +
                (current_pose.orientation.z - destination_pose.orientation.z) ** 2 +
                (current_pose.orientation.w - destination_pose.orientation.w) ** 2
            )

        return position_diff <= threshold and orientation_diff <= threshold


    def call_movement(self, waypoints):
        # Call InitWaypointSet service
        try:
            req = InitWaypointSetRequest()
            req.start_time = Time()  # Zero value by default
            req.start_now = True
            req.waypoints = waypoints
            req.max_forward_speed = 1.5
            req.heading_offset = 0.0
            req.interpolator = 'linear'
            response = self.init_waypoint_set_service(req)
            rospy.loginfo("InitWaypointSet service called.", response)

            if not response.success:
                rospy.logerr("Failed to initiate InitWaypointSet service.")
                return 'aborted'
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'
        

    def loop_monitor(self, userdata, waypoints):
        shared_data = userdata.shared_data
        # Monitoring loop
        while not rospy.is_shutdown():

            # Check if the destination has been reached
            if self.pose_reached(userdata.shared_data.current_pose,waypoints[0].point, threshold=self.threshold):
                rospy.loginfo("Destination has been reached.")
                return 'success'

            if self.edge_case_callback(shared_data):
                rospy.logwarn("Edge case detected, transitioning to handle situation.")
                userdata.edge_case = self.next_state_callback()
                return "edge_case_detected"

            rospy.sleep(0.1)  # Sleep to prevent a busy loop, adjust as needed

        return 'aborted'



    def execute(self, userdata):
        if userdata:
            shared_data = userdata.shared_data

        waypoints = self.generate_waypoints(self.num_waypoints)
        self.call_movement(waypoints)
        return self.loop_monitor(userdata)




         
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

        self.call_movement()
        return self.loop_monitor(userdata)



class Rotate90DegreesState(UpdatePoseState):
    def __init__(self, edge_case_callback,next_state_callback ):
        super(Rotate90DegreesState, self).__init__(outcomes=['success', 'edge_case_detected', 'aborted', "object_not_detected"],
                                                      input_keys=['shared_data'],
                                                      output_keys=['edge_case'],
                                                 edge_case_callback=edge_case_callback,
                                                   next_state_callback=next_state_callback)


    def execute(self, userdata):
        current_pose = userdata.shared_data.current_pose
        waypoint = Waypoint()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "world"
        waypoint.point.x = current_pose.position.x
        waypoint.point.y = current_pose.position.y
        waypoint.point.z = current_pose.position.z
        quaternion = tf.transformations.quaternion_from_euler(0, 0, math.pi/2)
        waypoint.orientation.x = quaternion[0]
        waypoint.orientation.y = quaternion[1] 
        waypoint.orientation.z = quaternion[2]
        waypoint.orientation.w = quaternion[3]

        self.waypoints = [waypoint] 
        self.call_movement(self.waypoints)
        return self.loop_monitor(userdata)



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