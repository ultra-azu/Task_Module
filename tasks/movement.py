#!/usr/bin/env python

import rospy
import smach
import smach_ros
from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped


class UpdatePoseState(smach.State):
    def __init__(self, group_name):
        smach.State.__init__(self, outcomes=['success', 'aborted'])
        self.group_name = group_name

    def execute(self, userdata):
        move_group = MoveGroupCommander(self.group_name)

        # Set the target pose for the end effector
        target_pose = PoseStamped()
        target_pose.header.frame_id = move_group.get_planning_frame()
        target_pose.pose.position.x = 0.5
        target_pose.pose.position.y = 0.5
        target_pose.pose.position.z = 0.5
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = 0.0
        target_pose.pose.orientation.w = 1.0

        # Plan and execute the movement
        move_group.set_pose_target(target_pose)
        plan = move_group.go(wait=True)
        if plan:
            return 'success'
        else:
            return 'aborted'

