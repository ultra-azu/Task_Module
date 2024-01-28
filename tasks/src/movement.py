import rospy
import smach
from uuv_control_msgs.srv import GoTo, GoToRequest
from geometry_msgs.msg import Pose, PoseStamped, Point
from uuv_control_msgs.msg import Waypoint
from uuv_control_msgs.srv import Hold, HoldRequest
from geometry_msgs.msg import PoseWithCovariance 
import rospy
import smach
import random
from std_msgs.msg import Time
from geometry_msgs.msg import Point
from uuv_control_msgs.srv import InitWaypointSet, InitWaypointSetRequest, GoTo, GoToRequest
from uuv_control_msgs.msg import Waypoint
import math
import tf
from data import read_yaml_file
import os
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class UpdatePoseState(smach.State):
    def __init__(self,  edge_case_callback,next_state_callback = None ,num_waypoints=1, pose = None ,threshold = 1.2,
                         speed = 2.0, heading_offset = 0.1, fixed_heading = False,radius_of_acceptance  = 0.5,):
        smach.State.__init__(self, outcomes=['success', 'edge_case_detected', 'aborted'],
                             input_keys=['shared_data'],
                             output_keys=['shared_data'])
        self.edge_case_callback = edge_case_callback
        self.next_state_callback = next_state_callback
        self.num_waypoints = num_waypoints
        self.pose = pose
        self.threshold = threshold
        self.speed = speed
        self.heading_offset = heading_offset
        self.fixed_heading = fixed_heading
        self.radius_of_acceptance = radius_of_acceptance


        # read from the config file the services
        services_topics = read_yaml_file(os.path.join(os.path.dirname(__file__), '../../config/topics.yaml'))
        self.init_waypoint_set_service = rospy.ServiceProxy(services_topics["uuv_control_services"]["init_waypoints"], InitWaypointSet)
        self.goto_service = rospy.ServiceProxy(services_topics["uuv_control_services"]["GoTo"], GoTo)

    @staticmethod
    def generate_waypoints(num_waypoints):
        waypoints = []
        for _ in range(num_waypoints):
            waypoint = Waypoint()
            waypoint.point = Point(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(-20, -10))
            waypoint.max_forward_speed = random.uniform(0, 5)
            waypoint.heading_offset = random.uniform(-3.14, 3.14)
            waypoint.use_fixed_heading = random.choice([True, False])
            waypoint.radius_of_acceptance = random.uniform(0, 1)
            waypoints.append(waypoint)
        return waypoints
    @staticmethod
    def WaypointFromPose(pose, speed, heading_offset, fixed_heading, radius_of_acceptance):
        waypoints = []
        waypoint = Waypoint()
        waypoint.point = pose.point
        waypoint.max_forward_speed = speed
        waypoint.heading_offset = heading_offset
        waypoint.use_fixed_heading = fixed_heading
        waypoint.radius_of_acceptance = radius_of_acceptance
        waypoints.append(waypoint)
        return waypoints
    
    @staticmethod
    def pose_reached( current_pose, destination_pose, threshold):
        # Check if the current pose is within a certain threshold of the destination pose
        # The function 'compare_poses' should return True if the poses are similar within the threshold
        # print("current_pose",current_pose)
        print(destination_pose)

        if not isinstance(current_pose, Odometry):
            rospy.logerr("current_pose must be an instance of Odometry")

        if not isinstance(destination_pose, Waypoint):
            rospy.logerr("destination_pose must be an instance of Waypoint")

        current_pose.pose.pose.position.x
        destination_pose.point.x
        position_diff = math.sqrt(
                (current_pose.pose.pose.position.x - destination_pose.point.x) ** 2 +
                (current_pose.pose.pose.position.y - destination_pose.point.y) ** 2 +
                (current_pose.pose.pose.position.z - destination_pose.point.z) ** 2
            )
        
        # TODO: Currently waypoints do not have an orientation data. Meaning that  we do not control that for now
                # To implement that we will need to chage the dp_controller.py file to include the orientation data.
                # and make a custom service and meesage for that information.
        

        
            # Calculate orientation difference (simple method, more complex calculations may involve quaternions)
        # orientation_diff = math.sqrt(
        #         (current_pose.orientation.x - destination_pose.orientation.x) ** 2 +
        #         (current_pose.orientation.y - destination_pose.orientation.y) ** 2 +
        #         (current_pose.orientation.z - destination_pose.orientation.z) ** 2 +
        #         (current_pose.orientation.w - destination_pose.orientation.w) ** 2
        #     )

        return position_diff <= threshold  #and orientation_diff <= threshold


    def call_movement(self, waypoints):
        # Call InitWaypointSet service
        try:
            req = InitWaypointSetRequest()
            req.start_time = Time()  # Zero value by default
            req.start_now = True
            req.waypoints = waypoints
            req.max_forward_speed = 1.5
            req.heading_offset = 0.0
            response = self.init_waypoint_set_service(req)
            rospy.loginfo("InitWaypointSet service called.")
            if not response.success:
                rospy.logerr("Failed to initiate InitWaypointSet service.")
                return 'aborted'
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'
        
    def call_goto_movement(self, waypoint):
        # Call the GoTo service
        try:
            req = GoToRequest()
            req.waypoint = waypoint
            req.max_forward_speed = 1.5
            response = self.goto_service(req)  # Replace with your service proxy name
            rospy.loginfo("GoTo service called.")

            if not response.success:
                rospy.logerr("Failed to initiate GoTo service.")
                return 'aborted'
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 'aborted'

    def loop_monitor(self, userdata, waypoints):
        shared_data = userdata.shared_data
        # Monitoring loop
        while not rospy.is_shutdown():

            # Check if the destination has been reached
            if self.pose_reached(userdata.shared_data.submarine_pose ,waypoints[0], threshold=self.threshold):
                rospy.loginfo("Destination has been reached.")
                return 'success'

            if self.edge_case_callback(shared_data):
                rospy.logwarn("Edge case detected, transitioning to handle situation.")
                return "edge_case_detected"

            rospy.sleep(0.1)  # Sleep to prevent a busy loop, adjust as needed
            print("Monitoring loop")

        return 'aborted'



    def execute(self, userdata):

        waypoints = self.generate_waypoints(self.num_waypoints)
        # print(waypoints)
        # waypoints = UpdatePoseState.WaypointFromPose( self.pose,  self.speed,self.heading_offset, 
        #                                                     self.radius_of_acceptance)
        result = self.call_goto_movement(waypoints[0])
        if result == 'aborted':
            return result
        return self.loop_monitor(userdata, waypoints)




         
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
        return self.loop_monitor(userdata, self.waypoints)



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

    def __init__(self, time_to_hold):
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